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
// =========================================================================
const float SPEED_SCALE            = 0.785f;
const float RIGHT_SPEED_CALIBRATION = 1.0f;
const float BASE_SPEED             = 0.71f;
const float MIN_DRIVE_DUTY         = 0.40f;  // floor for the faster wheel (raise if it stalls)
const float CURVE_SLOWDOWN         = 0.12f;  // base speed drops by this * |error|
 
// =========================================================================
// MOTOR DIRECTION CALIBRATION
// =========================================================================
const float LEFT_DIR_FORWARD  = 0.0f;
const float RIGHT_DIR_FORWARD = 0.0f;  // flip to 1.0f if the right wheel spins backward
 
// =========================================================================
// ADJUST STEERING (PD GAINS) HERE  -- these are the two knobs that matter
// =========================================================================
const float KP = 0.35f;
const float KD = 2.3f;
 
// =========================================================================
// OBSTACLE AVOIDANCE TUNING (NEW)  -- all the knobs for the side-step
// =========================================================================
const float OBSTACLE_DIST_CM = 26.5f;   
 
const int   STOP_MS          = 400;
 
const float TEST_LEFT        = 0.00f;   // left  wheel duty during the turn
const float TEST_RIGHT       = 0.7f;   // right wheel duty during the turn
const int   TEST_TIME_MS     = 680;     // hold time = your rotation angle
const int   RETURN_TIME_MS   = 1410;     // turn-back time (raise if it stays parallel)

const int   FWD_PAUSE_MS     = 1275;   // stop-and-wait after the forward drive,
                                      // before turning back toward the line
const float FWD_DUTY         = 0.70f;
const int   SEARCH_MAX_LOOPS = 300;   // ~3 s cap on the "drive until line" phase
const int   SONAR_EVERY      = 4;     // ping every 4 loops (~40 ms)
const float MIN_VALID_CM     = 2.0f;  // readings below this are noise -> ignored
const int   OBSTACLE_CONFIRM = 2;     // need this many CLOSE pings in a row to avoid

// =========================================================================
// OBSTACLE AVOIDANCE TUNING (DIAGONAL OBSTACLE)  -- all the knobs for the side-step
// =========================================================================

const float OBSTACLE_DIST_DIAG_CM = 20.0f;
const float TEST_LEFT_DIAG        = 0.0f;   // left  wheel duty during the turn
const float TEST_RIGHT_DIAG       = 0.7f;   // right wheel duty during the turn
const int   TEST_TIME_MS_DIAG     = 680;     // hold time = your rotation angle
const int   RETURN_TIME_MS_DIAG   = 1410; 
const float FWD_DUTY_DIAG         = 0.65f;
const int   FWD_PAUSE_MS_DIAG     = 1200;   // stop-and-wait after the forward drive

                                    
// =========================================================================
// SENSOR READING -> LINE POSITION
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
const int PRINT_EVERY = 10;
int print_counter = 0;
 
void set_motors(float left_speed, float right_speed) {
    if (left_speed  < 0.0f) left_speed  = 0.0f;
    if (left_speed  > 1.0f) left_speed  = 1.0f;
    if (right_speed < 0.0f) right_speed = 0.0f;
    if (right_speed > 1.0f) right_speed = 1.0f;
    left_motor_dir.write(RIGHT_DIR_FORWARD);
    right_motor_dir.write(LEFT_DIR_FORWARD);
    left_motor_speed.write(right_speed * SPEED_SCALE);
    right_motor_speed.write(left_speed * RIGHT_SPEED_CALIBRATION * SPEED_SCALE);
}

// Backward-move tuning (NEW)
const float BACKWARD_DUTY = 0.8f;   // reverse speed magnitude (0.0 .. 1.0)
const int   BACKWARD_MS   = 600;     // how long to back up
void set_motors_backward() {
     // Flip both direction pins to the reverse phase (cross-wired, as in set_motors).
    left_motor_dir.write(1.0f - RIGHT_DIR_FORWARD);
    right_motor_dir.write(1.0f - LEFT_DIR_FORWARD);

    // Same cross-wired speed assignment as set_motors().
    left_motor_speed.write(BACKWARD_DUTY * SPEED_SCALE);
    right_motor_speed.write(BACKWARD_DUTY * RIGHT_SPEED_CALIBRATION * SPEED_SCALE);

    ThisThread::sleep_for(std::chrono::milliseconds(BACKWARD_MS));

}

void set_motors_backward1(float left_speed, float right_speed) {
    if (left_speed  >  1.0f) left_speed  =  1.0f;
    if (left_speed  < -1.0f) left_speed  = -1.0f;
    if (right_speed >  1.0f) right_speed =  1.0f;
    if (right_speed < -1.0f) right_speed = -1.0f;

    float left_dir  = (left_speed  < 0.0f) ? (1.0f - LEFT_DIR_FORWARD)  : LEFT_DIR_FORWARD;
    float right_dir = (right_speed < 0.0f) ? (1.0f - RIGHT_DIR_FORWARD) : RIGHT_DIR_FORWARD;
    float left_mag  = (left_speed  < 0.0f) ? -left_speed  : left_speed;
    float right_mag = (right_speed < 0.0f) ? -right_speed : right_speed;

    left_motor_dir.write(right_dir);      // cross-wiring preserved
    right_motor_dir.write(left_dir);
    left_motor_speed.write(right_mag * SPEED_SCALE);
    right_motor_speed.write(left_mag * RIGHT_SPEED_CALIBRATION * SPEED_SCALE);
}


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
 
 
void stop_and_settle()
{
    set_motors(0.0f, 0.0f);
    ThisThread::sleep_for(std::chrono::milliseconds(150));
}
 
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
        if (t.elapsed_time() > 3ms) return 9999.0f;   // no object / no echo
    }
    Timer e;
    e.start();
    // measure how long echo stays HIGH
    while (echo.read() == 1) {
        if (e.elapsed_time() > 6ms) return 9999.0f;   // out of range
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

bool all_sensors_white() {
    return  ir_far_left.read()   > WHITE_THRESH &&
            ir_near_left.read()  > WHITE_THRESH &&
            ir_center.read()     > WHITE_THRESH &&
            ir_near_right.read() > WHITE_THRESH &&
            ir_far_right.read()  > WHITE_THRESH;
}

void rotation_on_all_white() {
    stop_and_settle();
    set_motors_backward1(-FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(700));
    stop_and_settle();

    set_motors_backward1(-FWD_DUTY, FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(400));
    stop_and_settle();

}
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
    set_motors_backward1(FWD_DUTY, -FWD_DUTY);
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

void avoid_obstacle_Diag() {
    // 1) stop + pause
    stop_and_settle();

    set_motors_backward1(-FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(775));
    stop_and_settle();
    
    set_motors_backward1(-FWD_DUTY, FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(TEST_TIME_MS_DIAG));
    stop_and_settle();

    // // 2) pivot away from the obstacle (toward the LEFT with these values)
    // set_motors(TEST_LEFT_DIAG, TEST_RIGHT_DIAG);
    // ThisThread::sleep_for(std::chrono::milliseconds(TEST_TIME_MS));
    // stop_and_settle();
 
    // 3) drive forward past the obstacle, then STOP AND WAIT
    set_motors(FWD_DUTY, FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(FWD_PAUSE_MS_DIAG));
    stop_and_settle();
 
    // 4) pivot back toward the line (mirror of step 2 -> toward the RIGHT)
    set_motors_backward1(FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(RETURN_TIME_MS_DIAG));
    stop_and_settle();

    // set_motors(FWD_DUTY_DIAG, FWD_DUTY_DIAG);
    // ThisThread::sleep_for(std::chrono::milliseconds(100));
    // stop_and_settle();

    // set_motors(TEST_LEFT_DIAG, TEST_RIGHT_DIAG);
    // ThisThread::sleep_for(std::chrono::milliseconds(350));
    // stop_and_settle();
 
    // 5) drive forward until the line is seen again (with a safety cap)
    set_motors(FWD_DUTY, FWD_DUTY);
    int guard = 0;
    while (guard < SEARCH_MAX_LOOPS) {
        if (all_sensors_white()) {
            rotation_on_all_white();
            break;   // rotation realigned us -> hand back to PD
            // Want it to keep searching after rotating instead of handing
            // back? Replace the break above with:
            //   set_motors(FWD_DUTY, FWD_DUTY);
        }
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
                    if (dist <  OBSTACLE_DIST_DIAG_CM) {
                        printf(">>> DIAGONAL obstacle at %d cm -> avoid_diagonal <<<\n", (int)dist);
                        avoid_obstacle_Diag();
                    } else {
                        printf(">>> OBSTACLE at %d cm -> avoid <<<\n", (int)dist);
                        avoid_obstacle();
                    }
                    
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
 
 
