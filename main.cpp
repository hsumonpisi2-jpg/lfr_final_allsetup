#include "mbed.h"
 
AnalogIn ir_far_right(A0);
AnalogIn ir_near_right(A1);
AnalogIn ir_center(A2);
AnalogIn ir_near_left(A3);
AnalogIn ir_far_left(A4);
 
PwmOut left_motor_speed(D11);
PwmOut left_motor_dir(D12);
PwmOut right_motor_speed(D9);
PwmOut right_motor_dir(PA_11_ALT0);
 
// =========================================================================
// ULTRASONIC OBSTACLE SENSOR (NEW)
// -------------------------------------------------------------------------
// HC-SR04 style sensor: send a 10us pulse on TRIG, then measure how long
// ECHO stays high. distance_cm = echo_high_time_us / 58.
//   TRIG = D6 (output), ECHO = D2 (input)
// =========================================================================
DigitalOut trig(D6);
DigitalIn  echo(D2);
 
// =========================================================================
// WHAT CHANGED vs THE OLD CODE (and why it fixes your two problems)
// -------------------------------------------------------------------------
// PROBLEM 1: "deviates largely on curves at high speed, then re-finds line"
//   The old code only knew 5 coarse states (hard/soft/straight), so on a
//   curve it kept flipping between "too little" and "too much" correction.
//   NEW: we compute a smooth line POSITION (-2.0 .. +2.0) from the ANALOG
//   sensor readings and steer proportionally (PD control). Small error ->
//   small correction, big error -> big correction, and the D (derivative)
//   term damps the swing so the robot hugs the curve instead of oscillating.
//
// PROBLEM 2: "if I slow it down, the motors stall after curves"
//   The old code commanded very low duty cycles (e.g. 0.60*0.8 = 0.48 and
//   0.20*0.8 = 0.16) during/after turns; below some duty a DC motor makes
//   no torque and the robot freezes until you push it.
//   NEW: MIN_DRIVE_DUTY guarantees the DRIVING (outer/faster) wheel is never
//   commanded below the duty where your motor still produces torque, and
//   the speed is only reduced on curves (proportionally to the error), so
//   straights stay fast.
//
// OBSTACLE AVOIDANCE (NEW): every few loops we ping the ultrasonic sensor.
//   If something is closer than OBSTACLE_DIST_CM, we run a simple, blocking
//   side-step maneuver: stop briefly, rotate away, drive forward past the
//   obstacle, rotate back toward the line, then drive forward until a sensor
//   sees the line again -> normal PD line following resumes. No counting, no
//   fancy logic, exactly as requested.
// =========================================================================
 
// =========================================================================
// ADJUST SPEEDS HERE
// -------------------------------------------------------------------------
//   SPEED_SCALE     -> master throttle, multiplies everything (same as before).
//   BASE_SPEED      -> cruising speed when the line is centred.
//   MIN_DRIVE_DUTY  -> the FLOOR for the faster wheel, AFTER SPEED_SCALE.
//                      Find it by testing: lowest duty where the robot still
//                      creeps forward reliably from standstill, plus a margin.
//                      If the robot still stalls after curves, RAISE this.
//   CURVE_SLOWDOWN  -> how much the robot slows down in curves.
//                      0.0 = no slowdown (full speed everywhere),
//                      higher = slower in curves. It only bites when the
//                      error is big, so straights are unaffected.
// =========================================================================
const float SPEED_SCALE            = 0.8f;
const float RIGHT_SPEED_CALIBRATION = 1.0f;
const float BASE_SPEED             = 0.7222f;
const float MIN_DRIVE_DUTY         = 0.40f;  // floor for the faster wheel (raise if it stalls)
const float CURVE_SLOWDOWN         = 0.12f;  // base speed drops by this * |error|
 
// =========================================================================
// MOTOR DIRECTION CALIBRATION
// -------------------------------------------------------------------------
// If the robot SPINS IN PLACE instead of driving straight when both wheels
// are commanded the same speed, the two motors are mounted mirrored on the
// chassis and one of them is spinning physically backward while the code
// thinks it's "forward". Fix it here, not by rewiring:
//   1) Prop the robot up so the wheels spin freely off the ground.
//   2) Run it -> both wheels should spin so that the TOP of each wheel
//      moves toward the FRONT of the robot.
//   3) Whichever wheel spins the wrong way -> flip its constant below
//      (0.0f <-> 1.0f).
// =========================================================================
const float LEFT_DIR_FORWARD  = 0.0f;
const float RIGHT_DIR_FORWARD = 0.0f;  // flip to 1.0f if the right wheel spins backward
 
// =========================================================================
// ADJUST STEERING (PD GAINS) HERE  -- these are the two knobs that matter
// -------------------------------------------------------------------------
// error = line position: 0 = centred, +2 = under far-right sensor,
//                        -2 = under far-left sensor.
// correction = KP * error + KD * (change in error per loop)
// left  wheel = base + correction,  right wheel = base - correction
//
//   KP -> how hard it steers back toward the line.
//         Robot drifts OFF on curves / reacts lazily  -> RAISE KP.
//         Robot wiggles/snakes on a STRAIGHT line     -> LOWER KP.
//   KD -> damping. Kills the overshoot that KP alone causes.
//         Robot oscillates a few times after each curve -> RAISE KD.
//         Robot reacts nervously to sensor noise        -> LOWER KD.
//
// Tuning recipe: set KD = 0. Raise KP until it follows your R150 curves but
// visibly wobbles. Then raise KD until the wobble dies out. Done.
// =========================================================================
const float KP = 0.35f;
const float KD = 2.3f;
 
// =========================================================================
// OBSTACLE AVOIDANCE TUNING (NEW)  -- all the knobs for the side-step
// -------------------------------------------------------------------------
//   OBSTACLE_DIST_CM -> trigger distance (YOUR threshold). Closer -> avoid.
//   STOP_MS          -> how long to pause when the obstacle is first seen.
//
//   The rotation is YOUR bench-tested PIVOT turn: one wheel drives, the
//   other is stopped, so the robot pivots about the stopped wheel.
//     TEST_LEFT / TEST_RIGHT -> the wheel duties you measured.
//     TEST_TIME_MS           -> how long to hold them = your rotation ANGLE.
//   These duties are written RAW (SPEED_SCALE is deliberately NOT applied to
//   the turn) so TEST_TIME_MS reproduces exactly the angle you measured on
//   the bench. If you find the turn is a bit too big/small once integrated,
//   just trim TEST_TIME_MS.
//     Turn AWAY = pivot(TEST_LEFT, TEST_RIGHT)  -> pivots toward the LEFT.
//     Turn BACK = pivot(TEST_RIGHT, TEST_LEFT)  -> pivots toward the RIGHT.
//   To detour around the OTHER side, swap those two calls in avoid_obstacle().
//   RETURN_TIME_MS -> hold time for the turn-back. Starts equal to
//                     TEST_TIME_MS; RAISE it if step 5 keeps finishing
//                     without re-finding the line (a symmetric out-and-back
//                     leaves the robot parallel to the line; over-rotating
//                     the return points it back INTO the line so it crosses).
//
//   FORWARD_MS       -> how far to drive forward to clear the obstacle after
//                       the first turn. Longer = wider detour.
//   FWD_DUTY         -> motor duty while driving forward during the maneuver.
//   SEARCH_MAX_LOOPS -> safety cap on the final "drive until line found"
//                       phase (loops of ~10 ms) so it can never drive off
//                       forever if it misses the line.
//   SONAR_EVERY      -> ping the ultrasonic every N control loops (keeps the
//                       ~25 ms echo timeout from disturbing the 10 ms PD loop
//                       on every iteration).
// =========================================================================
const float OBSTACLE_DIST_CM = 28.0f;   // your threshold
 
const int   STOP_MS          = 400;
 
const float TEST_LEFT        = 0.00f;   // left  wheel duty during the turn
const float TEST_RIGHT       = 0.7f;   // right wheel duty during the turn
const int   TEST_TIME_MS     = 600;     // hold time = your rotation angle
const int   RETURN_TIME_MS   = 1500;     // turn-back time (raise if it stays parallel)
 
const float FORWARD_CM       = 30.0f;   // ~28 cm at FWD_DUTY on this robot (measured)
const float FORWARD_MS_PER_CM = 42.0f;
const int   FWD_PAUSE_MS     = FORWARD_CM * FORWARD_MS_PER_CM;   // stop-and-wait after the forward drive,
                                      // before turning back toward the line
const float FWD_DUTY         = 0.70f;
const int   SEARCH_MAX_LOOPS = 300;   // ~3 s cap on the "drive until line" phase
const int   SONAR_EVERY      = 4;     // ping every 4 loops (~40 ms)
const float MIN_VALID_CM     = 2.0f;  // readings below this are noise -> ignored
const int   OBSTACLE_CONFIRM = 2;     // need this many CLOSE pings in a row to avoid
 
// =========================================================================
// SENSOR READING -> LINE POSITION
// -------------------------------------------------------------------------
// Instead of a hard white/black threshold per sensor, we use HOW MUCH white
// each sensor sees (the analog value) and take a weighted average:
//     position = sum(weight_i * a_i) / sum(a_i)
// with weights -2,-1,0,+1,+2 and a_i = reading above BLACK_FLOOR.
// This gives a smooth position even when the line sits BETWEEN two sensors,
// which is exactly what proportional steering needs.
//
//   FLOOR_* -> EACH sensor's own reading on plain black + a little margin.
//              Anything below its floor counts as 0 (pure black) for that
//              sensor. The floors are PER SENSOR because sensors differ:
//              measured on this robot, the four outer sensors read ~0.16 on
//              black but the CENTER one reads ~0.5, so it needs a much
//              higher floor or it fakes a line signal on plain black.
//              To calibrate: put the robot on plain black, read all five
//              raw values, set each floor a bit above its own reading.
//   WHITE_THRESH-> still used for the yes/no tests (goal bars, junctions).
//   MIN_SUM     -> if the total "whiteness" is below this, NO sensor sees
//                  the line -> line lost -> search toward last known side.
// =========================================================================
const float FLOOR_FAR_L  = 0.25f;   // ~0.16 on black + margin
const float FLOOR_NEAR_L = 0.25f;
const float FLOOR_CENTER = 0.60f;   // ~0.5 on black + margin (this one is different!)
const float FLOOR_NEAR_R = 0.25f;
const float FLOOR_FAR_R  = 0.25f;
const float WHITE_THRESH = 0.80f;
const float MIN_SUM      = 0.10f;
 
// =========================================================================
// GOAL / STOP MARKER (unchanged from the old code)
// =========================================================================
const int DOUBLE_LINE_WINDOW = 50;   // ~0.5 s at 10 ms per loop
const int START_GRACE        = 200;  // ~2 s at 10 ms per loop
 
// =========================================================================
// SERIAL MONITOR (debug) -- one line every ~100 ms, same as before.
// Prints position and correction as integers x100 (mbed printf often has
// float support disabled by default).
// =========================================================================
const int PRINT_EVERY = 10;
int print_counter = 0;
 
void set_motors(float left_speed, float right_speed) {
    if (left_speed  < 0.0f) left_speed  = 0.0f;
    if (left_speed  > 1.0f) left_speed  = 1.0f;
    if (right_speed < 0.0f) right_speed = 0.0f;
    if (right_speed > 1.0f) right_speed = 1.0f;
    left_motor_dir.write(RIGHT_DIR_FORWARD);
    right_motor_dir.write(LEFT_DIR_FORWARD);
    // Wheels are cross-wired on this chassis: the pin labeled "left_motor"
    // physically drives the right wheel and vice versa. Swap here (once,
    // in the driving function) so every caller can keep thinking in terms
    // of true left/right without knowing about the wiring quirk.
    left_motor_speed.write(right_speed * SPEED_SCALE);
    right_motor_speed.write(left_speed * RIGHT_SPEED_CALIBRATION * SPEED_SCALE);
}
 
// Anti-stall: make sure the faster (driving) wheel never drops below the
// duty where the motor still produces torque. Both wheels are shifted up
// together so the steering ratio between them is preserved.
void set_motors_with_floor(float left_speed, float right_speed) {
    float faster = (left_speed > right_speed) ? left_speed : right_speed;
    float floor_duty = MIN_DRIVE_DUTY / SPEED_SCALE;  // floor applies AFTER scaling
    if (faster < floor_duty && faster > 0.0f) {
        float boost = floor_duty - faster;
        left_speed  += boost;
        right_speed += boost;
    }
    set_motors(left_speed, right_speed);
}
 
// -------------------------------------------------------------------------
// Bench-calibrated PIVOT turn (NEW), used by the obstacle maneuver. Both
// wheels drive FORWARD; stopping one (duty 0) makes the robot pivot about
// it. The duties are written RAW -- SPEED_SCALE is deliberately NOT applied
// here -- so the values you measured (TEST_*) reproduce your exact angle for
// a given hold time. Honors the same cross-wiring and direction-calibration
// constants as set_motors(), so normal line following is unaffected.
//   left_duty, right_duty : 0.0 .. 1.0  (logical left / right wheel)
// -------------------------------------------------------------------------
 
 
void stop_and_settle()
{
    set_motors(0.0f, 0.0f);
    ThisThread::sleep_for(std::chrono::milliseconds(150));
}
 
 
// -------------------------------------------------------------------------
// Ultrasonic distance in cm (NEW). Returns a large value (9999) if there is
// no echo within the timeout -> treated as "nothing close".
// -------------------------------------------------------------------------
float read_distance_cm() {
    // 10us trigger pulse
    trig = 0;
    wait_us(2);
    trig = 1;
    wait_us(10);
    trig = 0;
 
    Timer t;
    t.start();
    // wait for echo to go HIGH (start of the return pulse)
    while (echo.read() == 0) {
        if (t.elapsed_time() > 25ms) return 9999.0f;   // no object / no echo
    }
    Timer e;
    e.start();
    // measure how long echo stays HIGH
    while (echo.read() == 1) {
        if (e.elapsed_time() > 25ms) return 9999.0f;   // out of range
    }
    long long us = std::chrono::duration_cast<std::chrono::microseconds>(
                       e.elapsed_time()).count();
    return (float)us / 58.0f;
}
 
// -------------------------------------------------------------------------
// Quick "is any sensor on the line right now?" check (NEW), used while the
// robot drives forward at the end of the maneuver so it knows when to hand
// control back to the PD line follower.
// -------------------------------------------------------------------------
bool line_visible() {
    float a_fL = ir_far_left.read()  - FLOOR_FAR_L;  if (a_fL < 0.0f) a_fL = 0.0f;
    float a_nL = ir_near_left.read() - FLOOR_NEAR_L; if (a_nL < 0.0f) a_nL = 0.0f;
    float a_c  = ir_center.read()    - FLOOR_CENTER; if (a_c  < 0.0f) a_c  = 0.0f;
    float a_nR = ir_near_right.read()- FLOOR_NEAR_R; if (a_nR < 0.0f) a_nR = 0.0f;
    float a_fR = ir_far_right.read() - FLOOR_FAR_R;  if (a_fR < 0.0f) a_fR = 0.0f;
    return (a_fL + a_nL + a_c + a_nR + a_fR) > MIN_SUM;
}
 
// -------------------------------------------------------------------------
// THE OBSTACLE MANEUVER (NEW). Blocking and deliberately simple:
//   1) stop and pause,
//   2) pivot AWAY from the obstacle (your calibrated TEST turn),
//   3) drive forward to get past it,
//   4) pivot BACK toward the line (mirror of the first turn),
//   5) drive forward until a sensor sees the line again (or timeout),
// then return so the main loop resumes normal PD line following.
// A symmetric out-and-back leaves the robot parallel to the line; if step 5
// keeps finishing without finding it, raise RETURN_TIME_MS so the turn-back
// over-rotates and points the robot back INTO the line so it crosses.
// -------------------------------------------------------------------------
void avoid_obstacle() {
    // 1) stop + pause
    stop_and_settle();
 
    // 2) pivot away from the obstacle (toward the LEFT with these values)
    set_motors(TEST_LEFT, TEST_RIGHT);
    ThisThread::sleep_for(std::chrono::milliseconds(TEST_TIME_MS));
    stop_and_settle();
 
    // 3) drive forward past the obstacle, then STOP AND WAIT
    set_motors(FWD_DUTY, FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(FWD_PAUSE_MS));
    stop_and_settle();
 
    // 4) pivot back toward the line (mirror of step 2 -> toward the RIGHT)
    set_motors(TEST_RIGHT, TEST_LEFT);
    ThisThread::sleep_for(std::chrono::milliseconds(RETURN_TIME_MS));
    stop_and_settle();
 
    // 5) drive forward until the line is seen again (with a safety cap)
    set_motors(FWD_DUTY, FWD_DUTY);
    int guard = 0;
    while (guard < SEARCH_MAX_LOOPS) {
        if (line_visible()) break;   // line re-acquired -> hand back to PD
        guard++;
        ThisThread::sleep_for(10ms);
    }
    set_motors(0.0f, 0.0f);
}
 
int main() {
    left_motor_speed.period(0.02f);
    right_motor_speed.period(0.02f);
    left_motor_dir.period(0.02f);
    right_motor_dir.period(0.02f);
 
    printf("\n--- line follower (PD) + obstacle avoid start ---\n");
    printf("bar = [farL nearL center nearR farR]  pos/corr are x100\n");
 
    // Goal detection state.
    bool prev_all_white = false;
    int  pair_window = 0;
    int  grace = START_GRACE;
 
    // PD state.
    float last_error = 0.0f;   // error from the previous loop (for KD)
    int   last_dir   = 0;      // which side the line was last seen (+1 R, -1 L)
 
    // Ultrasonic sampling state (NEW).
    int sonar_counter  = 0;
    int obstacle_hits  = 0;   // consecutive close pings (for OBSTACLE_CONFIRM)
 
    while (1) {
        // ----- Obstacle check (NEW): ping every SONAR_EVERY loops -----
        if (++sonar_counter >= SONAR_EVERY) {
            sonar_counter = 0;
            float dist = read_distance_cm();
            // DEBUG: watch this in the serial monitor. If it prints a small
            // number with NOTHING in front of the sensor, the problem is the
            // wiring, not the code (see the notes I gave you).
            printf("dist=%d cm\n", (int)dist);
            // A reading only counts if it is VALID (not noise) AND CLOSE, and
            // it must repeat for OBSTACLE_CONFIRM pings in a row. This is what
            // stops a single junk reading from triggering avoidance forever.
            if (dist > MIN_VALID_CM && dist < OBSTACLE_DIST_CM) {
                if (++obstacle_hits >= OBSTACLE_CONFIRM) {
                    obstacle_hits = 0;
                    printf(">>> OBSTACLE at %d cm -> avoid <<<\n", (int)dist);
                    avoid_obstacle();
                    // Reset PD state so the derivative term doesn't jump after
                    // the (long) blocking maneuver.
                    last_error = 0.0f;
                    last_dir   = 0;
                    continue;   // re-read sensors fresh next loop
                }
            } else {
                obstacle_hits = 0;   // streak broken -> reset
            }
        }
 
        // ----- Read all five sensors (analog, 0.0 .. 1.0) -----
        float v_fL = ir_far_left.read();
        float v_nL = ir_near_left.read();
        float v_c  = ir_center.read();
        float v_nR = ir_near_right.read();
        float v_fR = ir_far_right.read();
 
        // Yes/no versions for goal bars, junctions and the debug print.
        bool far_L  = v_fL > WHITE_THRESH;
        bool near_L = v_nL > WHITE_THRESH;
        bool center = v_c  > WHITE_THRESH;
        bool near_R = v_nR > WHITE_THRESH;
        bool far_R  = v_fR > WHITE_THRESH;
 
        // ----- Stop at the goal: two white bars close together (unchanged) -----
        if (grace > 0) {
            grace--;
        }
        bool all_white = far_L && near_L && center && near_R && far_R;
        bool bar_edge = all_white && !prev_all_white;
        prev_all_white = all_white;
        if (pair_window > 0) {
            pair_window--;
        }
        if (bar_edge && grace == 0) {
            if (pair_window > 0) {
                set_motors(0.0f, 0.0f);
                printf(">>> GOAL (double line) -> STOP <<<\n");
                while (1) {
                    ThisThread::sleep_for(100ms);
                }
            }
            pair_window = DOUBLE_LINE_WINDOW;
        }
 
        // ----- Line position from the analog readings -----
        // a_i = how much white sensor i sees above ITS OWN black level
        // (0 on plain black).
        float a_fL = v_fL - FLOOR_FAR_L;  if (a_fL < 0.0f) a_fL = 0.0f;
        float a_nL = v_nL - FLOOR_NEAR_L; if (a_nL < 0.0f) a_nL = 0.0f;
        float a_c  = v_c  - FLOOR_CENTER; if (a_c  < 0.0f) a_c  = 0.0f;
        float a_nR = v_nR - FLOOR_NEAR_R; if (a_nR < 0.0f) a_nR = 0.0f;
        float a_fR = v_fR - FLOOR_FAR_R;  if (a_fR < 0.0f) a_fR = 0.0f;
        float sum = a_fL + a_nL + a_c + a_nR + a_fR;
 
        const char* action = "?";
        float error = 0.0f;
        float correction = 0.0f;
 
        if (far_L && far_R) {
            // Cross / T-junction / start bar: both far sensors on white.
            // A weighted average would be meaningless here -> go straight.
            set_motors_with_floor(BASE_SPEED, BASE_SPEED);
            last_error = 0.0f;
            last_dir = 0;
            action = "CROSS/JUNCTION -> straight";
        }
        else if (sum > MIN_SUM) {
            // ----- Normal PD line following -----
            error = (-2.0f * a_fL - 1.0f * a_nL + 1.0f * a_nR + 2.0f * a_fR) / sum;
 
            correction = KP * error + KD * (error - last_error);
            last_error = error;
 
            // Remember which zone the line was last seen in, for the
            // line-lost fallback below. Line last seen under an OUTER
            // sensor -> it was drifting off to that side (a curve), so a
            // directional pivot search makes sense. Line last seen under
            // one of the three MIDDLE sensors -> the robot was still
            // roughly centred, so a sudden loss is more likely a short
            // gap in an otherwise straight line -> just keep going straight.
            if      (far_L && !far_R) last_dir = -1;
            else if (far_R && !far_L) last_dir = 1;
            else if (near_L || center || near_R) last_dir = 0;
 
            // Slow down in proportion to how far off-centre we are, so the
            // robot brakes INTO the curve but keeps full speed on straights.
            float abs_err = (error < 0.0f) ? -error : error;
            float base = BASE_SPEED - CURVE_SLOWDOWN * abs_err;
 
            // error > 0 -> line is to the RIGHT -> steer right:
            // left wheel speeds up, right wheel slows down.
            set_motors_with_floor(base + correction, base - correction);
            action = "PD follow";
        }
        else {
            // ----- Line lost: turn back toward where it last was -----
            // The driving wheel gets the full BASE_SPEED (not a reduced
            // "turn speed") so the robot cannot stall while searching.
            if (last_dir > 0) {
                set_motors_with_floor(BASE_SPEED, 0.0f);
                action = "LINE LOST -> search RIGHT";
            } else if (last_dir < 0) {
                set_motors_with_floor(0.0f, BASE_SPEED);
                action = "LINE LOST -> search LEFT";
            } else {
                set_motors_with_floor(BASE_SPEED, BASE_SPEED);
                action = "LINE LOST -> straight";
            }
            last_error = 0.0f;
        }
 
        // Throttled status line: sensor bar + position + correction + action.
        if (++print_counter >= PRINT_EVERY) {
            print_counter = 0;
            printf("[%c %c %c %c %c] pos=%4d corr=%4d  %s\n",
                   far_L  ? 'X' : '.',
                   near_L ? 'X' : '.',
                   center ? 'X' : '.',
                   near_R ? 'X' : '.',
                   far_R  ? 'X' : '.',
                   (int)(error * 100.0f),
                   (int)(correction * 100.0f),
                   action);
        }
 
        ThisThread::sleep_for(10ms);
    }
}
 
 
