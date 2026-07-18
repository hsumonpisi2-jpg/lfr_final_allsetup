#include "mbed.h"

// ============================================================================
//  LINE FOLLOWER (PD) + ULTRASONIC OBSTACLE AVOIDANCE
//  Nucleo-F303K8 / Mbed
// ----------------------------------------------------------------------------
//  Behaviour overview:
//   - 5 analog IR sensors give a smooth line POSITION (-2.0 .. +2.0); the
//     robot steers proportionally with PD control (KP/KD).
//   - Speed drops in proportion to how far off-centre it is, so it brakes
//     into curves but keeps full speed on straights. MIN_DRIVE_DUTY keeps
//     the faster wheel above its stall duty.
//   - The HC-SR04 pings every SONAR_EVERY loops; a confirmed close reading
//     runs a blocking side-step maneuver (frontal or diagonal), then hands
//     control back to PD line following.
//
//  DETECTION FIXES (diagonal-approach obstacle):
//   FIX A: confirmation streak DECAYS on a bad reading instead of resetting
//          to zero, so flickery valid/9999 readings on the angled approach
//          can no longer block the trigger until impact.
//   FIX B: EMERGENCY_DIST_CM band -- one valid reading closer than this
//          triggers avoidance immediately, no confirmation streak needed.
//   FIX C: SONAR_EVERY = 2 (ping ~every 20 ms) and the dist printf is
//          throttled so it no longer adds UART blocking to every ping.
//
//  >>> ALL TUNING KNOBS ARE IN THE "TUNING CONFIG" BLOCK BELOW <<<
// ============================================================================


// ============================================================================
//  HARDWARE PINS
// ============================================================================
AnalogIn ir_far_right (A0);
AnalogIn ir_near_right(A1);
AnalogIn ir_center    (A2);
AnalogIn ir_near_left (A3);
AnalogIn ir_far_left  (A4);

PwmOut left_motor_speed (D11);
PwmOut left_motor_dir   (D12);
PwmOut right_motor_speed(D9);
PwmOut right_motor_dir  (PA_11_ALT0);

DigitalOut trig(D6);   // HC-SR04 TRIG (output)
DigitalIn  echo(D2);   // HC-SR04 ECHO (input)


// ############################################################################
// #                                                                          #
// #                          TUNING CONFIG                                   #
// #                     adjust everything here                               #
// #                                                                          #
// ############################################################################

// --- Speed ------------------------------------------------------------------
const float SPEED_SCALE             = 0.785f;  // master duty multiplier
const float RIGHT_SPEED_CALIBRATION = 1.0f;    // trim if one wheel runs faster
const float BASE_SPEED              = 0.71f;   // straight-line cruise duty
const float MIN_DRIVE_DUTY          = 0.40f;   // floor for faster wheel (raise if it stalls)
const float CURVE_SLOWDOWN          = 0.12f;   // base speed drops by this * |error|

// --- Motor direction --------------------------------------------------------
const float LEFT_DIR_FORWARD        = 0.0f;
const float RIGHT_DIR_FORWARD       = 0.0f;    // flip to 1.0f if right wheel spins backward

// --- PD steering gains (the two knobs that matter most) ---------------------
const float KP = 0.35f;
const float KD = 2.3f;

// --- Backward move ----------------------------------------------------------
const float BACKWARD_DUTY = 0.8f;              // reverse speed magnitude (0.0 .. 1.0)
const int   BACKWARD_MS   = 600;               // how long to back up

// --- Obstacle avoidance : FRONTAL -------------------------------------------
const float OBSTACLE_DIST_CM = 26.5f;          // trigger distance (cm)
const int   STOP_MS          = 400;
const float TEST_LEFT        = 0.00f;          // left  wheel duty during the pivot
const float TEST_RIGHT       = 0.70f;          // right wheel duty during the pivot
const int   TEST_TIME_MS     = 680;            // pivot hold time = rotation angle
const int   RETURN_TIME_MS   = 1410;           // turn-back time (raise if it stays parallel)
const int   FWD_PAUSE_MS     = 1275;           // forward drive time past the obstacle
const float FWD_DUTY         = 0.70f;          // forward duty during the maneuver

// --- Obstacle avoidance : DIAGONAL ------------------------------------------
const float OBSTACLE_DIST_DIAG_CM = 20.0f;     // switch to diagonal below this (cm)
const float TEST_LEFT_DIAG        = 0.0f;      // (currently unused – see avoid_obstacle_Diag)
const float TEST_RIGHT_DIAG       = 0.7f;      // (currently unused)
const int   TEST_TIME_MS_DIAG     = 680;
const int   RETURN_TIME_MS_DIAG   = 1410;
const float FWD_DUTY_DIAG         = 0.65f;     // (currently unused)
const int   FWD_PAUSE_MS_DIAG     = 1200;

// --- Obstacle detection (sonar) ---------------------------------------------
// FIX B: panic band. ONE valid reading below this triggers avoidance
// immediately, skipping the confirmation streak. Sensor sits ~5 cm behind
// the nose, so 12 cm at the sensor = ~7 cm at the bumper.
const float EMERGENCY_DIST_CM = 12.0f;

// FIX C: ping twice as often (~every 20 ms instead of ~40 ms).
const int   SONAR_EVERY      = 2;              // ping every N loops (was 4)
const float MIN_VALID_CM     = 2.0f;           // ignore readings below this (noise)
const int   OBSTACLE_CONFIRM = 2;              // consecutive close pings needed to trigger
const int   SONAR_PRINT_EVERY = 10;            // FIX C: print dist only every Nth ping
const int   SEARCH_MAX_LOOPS = 300;            // ~3 s cap on the "drive until line" phase

// --- Sensor floors (black-level per sensor) & thresholds --------------------
const float FLOOR_FAR_L  = 0.25f;
const float FLOOR_NEAR_L = 0.25f;
const float FLOOR_CENTER = 0.60f;              // center reads higher on black than the rest
const float FLOOR_NEAR_R = 0.25f;
const float FLOOR_FAR_R  = 0.25f;
const float WHITE_THRESH = 0.80f;              // above this = sensor sees white
const float MIN_SUM      = 0.10f;              // below this total = line lost

// --- Goal marker / timing ---------------------------------------------------
const int DOUBLE_LINE_WINDOW = 50;             // ~0.5 s window to see the second bar
const int START_GRACE        = 200;            // ~2 s startup ignore for goal detection
const int PRINT_EVERY        = 10;             // throttle serial prints

// ############################################################################
// #                        END TUNING CONFIG                                 #
// ############################################################################


// ============================================================================
//  MOTOR HELPERS
// ============================================================================

// Forward drive. Cross-wiring is intentional: left/right speed and dir are
// swapped to match how the driver is wired.
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

// Straight reverse for BACKWARD_MS at BACKWARD_DUTY (blocking).
void set_motors_backward() {
    left_motor_dir.write(1.0f - RIGHT_DIR_FORWARD);
    right_motor_dir.write(1.0f - LEFT_DIR_FORWARD);
    left_motor_speed.write(BACKWARD_DUTY * SPEED_SCALE);
    right_motor_speed.write(BACKWARD_DUTY * RIGHT_SPEED_CALIBRATION * SPEED_SCALE);
    ThisThread::sleep_for(std::chrono::milliseconds(BACKWARD_MS));
}

// Signed drive: negative speed = that wheel runs in reverse. Used for pivots
// and reversing during the obstacle maneuvers. Cross-wiring preserved.
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

// Forward drive with a stall floor: if the faster wheel would be below
// MIN_DRIVE_DUTY, boost both wheels equally so it still produces torque.
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

void stop_and_settle() {
    set_motors(0.0f, 0.0f);
    ThisThread::sleep_for(std::chrono::milliseconds(150));
}


// ============================================================================
//  SENSOR HELPERS
// ============================================================================

// HC-SR04: 10us trigger pulse, then measure echo-high time. cm = us / 58.
float read_distance_cm() {
    trig = 0;
    wait_us(2);
    trig = 1;
    wait_us(10);
    trig = 0;

    Timer t;
    t.start();
    while (echo.read() == 0) {                       // wait for echo HIGH
        if (t.elapsed_time() > 3ms) return 9999.0f;  // no object / no echo
    }
    Timer e;
    e.start();
    while (echo.read() == 1) {                       // measure echo HIGH time
        if (e.elapsed_time() > 6ms) return 9999.0f;  // out of range
    }
    long long us = std::chrono::duration_cast<std::chrono::microseconds>(
                       e.elapsed_time()).count();
    return (float)us / 58.0f;
}

// "Is any sensor on the line right now?" — used during the drive-until-line
// phase to know when to hand control back to PD.
bool line_visible() {
    float a_fL = ir_far_left.read()  - FLOOR_FAR_L;  if (a_fL < 0.0f) a_fL = 0.0f;
    float a_nL = ir_near_left.read() - FLOOR_NEAR_L; if (a_nL < 0.0f) a_nL = 0.0f;
    float a_c  = ir_center.read()    - FLOOR_CENTER; if (a_c  < 0.0f) a_c  = 0.0f;
    float a_nR = ir_near_right.read()- FLOOR_NEAR_R; if (a_nR < 0.0f) a_nR = 0.0f;
    float a_fR = ir_far_right.read() - FLOOR_FAR_R;  if (a_fR < 0.0f) a_fR = 0.0f;
    return (a_fL + a_nL + a_c + a_nR + a_fR) > MIN_SUM;
}

bool all_sensors_white() {
    return ir_far_left.read()   > WHITE_THRESH &&
           ir_near_left.read()  > WHITE_THRESH &&
           ir_center.read()     > WHITE_THRESH &&
           ir_near_right.read() > WHITE_THRESH &&
           ir_far_right.read()  > WHITE_THRESH;
}


// ============================================================================
//  OBSTACLE MANEUVERS
//  Blocking side-steps: pivot away, drive past, pivot back, then drive
//  forward until a sensor re-acquires the line.
// ============================================================================

// Recovery when the diagonal search hits an all-white patch: back up, then
// pivot to realign onto the line.
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

    // 3) drive forward past the obstacle, then stop and wait
    set_motors(FWD_DUTY, FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(FWD_PAUSE_MS));
    stop_and_settle();

    // 4) pivot back toward the line (mirror of step 2 -> toward the RIGHT)
    set_motors_backward1(FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(RETURN_TIME_MS));
    stop_and_settle();

    // 5) drive forward until the line is seen again (safety-capped)
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

    // 2) back up, then pivot away from the diagonal obstacle
    set_motors_backward1(-FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(775));
    stop_and_settle();

    set_motors_backward1(-FWD_DUTY, FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(TEST_TIME_MS_DIAG));
    stop_and_settle();

    // 3) drive forward past the obstacle, then stop and wait
    set_motors(FWD_DUTY, FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(FWD_PAUSE_MS_DIAG));
    stop_and_settle();

    // 4) pivot back toward the line
    set_motors_backward1(FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(RETURN_TIME_MS_DIAG));
    stop_and_settle();

    // 5) drive forward until the line is seen again (safety-capped).
    //    If it hits an all-white patch first, run the realign rotation.
    set_motors(FWD_DUTY, FWD_DUTY);
    int guard = 0;
    while (guard < SEARCH_MAX_LOOPS) {
        if (all_sensors_white()) {
            rotation_on_all_white();
            break;                   // realigned -> hand back to PD
        }
        if (line_visible()) break;   // line re-acquired -> hand back to PD
        guard++;
        ThisThread::sleep_for(10ms);
    }
    set_motors(0.0f, 0.0f);
}


// ============================================================================
//  MAIN LOOP
// ============================================================================
int main() {
    left_motor_speed.period(0.02f);
    right_motor_speed.period(0.02f);
    left_motor_dir.period(0.02f);
    right_motor_dir.period(0.02f);

    printf("\n--- line follower (PD) + obstacle avoid start ---\n");
    printf("bar = [farL nearL center nearR farR]  pos/corr are x100\n");

    // Goal detection state.
    bool prev_all_white = false;
    int  pair_window    = 0;
    int  grace          = START_GRACE;

    // PD state.
    float last_error = 0.0f;   // previous-loop error (for KD)
    int   last_dir   = 0;      // side line was last seen (+1 R, -1 L)

    // Ultrasonic sampling state.
    int sonar_counter     = 0;
    int obstacle_hits     = 0;  // confirmation streak (decays, never hard-reset)
    int sonar_print_count = 0;  // FIX C: throttle the dist printf
    int print_counter     = 0;

    while (1) {
        // ----- Obstacle check: ping every SONAR_EVERY loops -----
        if (++sonar_counter >= SONAR_EVERY) {
            sonar_counter = 0;
            float dist = read_distance_cm();

            // FIX C: print only every SONAR_PRINT_EVERY-th ping, OR whenever
            // the reading is in the interesting (close) range. A printf on
            // every ping was adding blocking UART latency to the loop.
            bool close = (dist > MIN_VALID_CM && dist < OBSTACLE_DIST_CM);
            if (close || ++sonar_print_count >= SONAR_PRINT_EVERY) {
                sonar_print_count = 0;
                printf("dist=%d cm\n", (int)dist);
            }

            if (close) {
                // FIX B: EMERGENCY band -- a valid reading already inside
                // EMERGENCY_DIST_CM triggers immediately, no streak needed.
                // On the diagonal approach the angled face returns its FIRST
                // valid echo very late and very close; waiting for 2-in-a-row
                // was exactly what let the robot ram the obstacle.
                bool emergency = (dist < EMERGENCY_DIST_CM);

                if (emergency || ++obstacle_hits >= OBSTACLE_CONFIRM) {
                    obstacle_hits = 0;
                    if (dist < OBSTACLE_DIST_DIAG_CM) {
                        printf(">>> DIAGONAL obstacle at %d cm -> avoid_diagonal <<<\n", (int)dist);
                        avoid_obstacle_Diag();
                    } else {
                        printf(">>> OBSTACLE at %d cm -> avoid <<<\n", (int)dist);
                        avoid_obstacle();
                    }
                    // Reset PD state so KD doesn't jump after the blocking maneuver.
                    last_error = 0.0f;
                    last_dir   = 0;
                    continue;   // re-read sensors fresh next loop
                }
            } else {
                // FIX A: DECAY the streak instead of wiping it. On the angled
                // approach readings flicker valid/9999/valid; a hard reset
                // meant the confirmation could never finish until the robot
                // was already touching the obstacle.
                if (obstacle_hits > 0) obstacle_hits--;
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

        // ----- Stop at the goal: two white bars close together -----
        if (grace > 0) grace--;
        bool all_white = far_L && near_L && center && near_R && far_R;
        bool bar_edge  = all_white && !prev_all_white;
        prev_all_white = all_white;
        if (pair_window > 0) pair_window--;
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
        // a_i = how much white sensor i sees above ITS OWN black level.
        float a_fL = v_fL - FLOOR_FAR_L;  if (a_fL < 0.0f) a_fL = 0.0f;
        float a_nL = v_nL - FLOOR_NEAR_L; if (a_nL < 0.0f) a_nL = 0.0f;
        float a_c  = v_c  - FLOOR_CENTER; if (a_c  < 0.0f) a_c  = 0.0f;
        float a_nR = v_nR - FLOOR_NEAR_R; if (a_nR < 0.0f) a_nR = 0.0f;
        float a_fR = v_fR - FLOOR_FAR_R;  if (a_fR < 0.0f) a_fR = 0.0f;
        float sum  = a_fL + a_nL + a_c + a_nR + a_fR;

        const char* action = "?";
        float error      = 0.0f;
        float correction = 0.0f;

        if (far_L && far_R) {
            // Cross / T-junction / start bar: both far sensors on white -> straight.
            set_motors_with_floor(BASE_SPEED, BASE_SPEED);
            last_error = 0.0f;
            last_dir   = 0;
            action = "CROSS/JUNCTION -> straight";
        }
        else if (sum > MIN_SUM) {
            // ----- Normal PD line following -----
            error = (-2.0f * a_fL - 1.0f * a_nL + 1.0f * a_nR + 2.0f * a_fR) / sum;

            correction = KP * error + KD * (error - last_error);
            last_error = error;

            // Remember which zone the line was last in, for the line-lost
            // fallback. Outer sensor -> drifting off that side (curve) ->
            // directional search. Middle sensors -> still roughly centred ->
            // a loss is likely a gap -> keep straight.
            if      (far_L && !far_R) last_dir = -1;
            else if (far_R && !far_L) last_dir = 1;
            else if (near_L || center || near_R) last_dir = 0;

            // Slow down proportionally to how far off-centre we are.
            float abs_err = (error < 0.0f) ? -error : error;
            float base = BASE_SPEED - CURVE_SLOWDOWN * abs_err;

            // error > 0 -> line is RIGHT -> steer right (left wheel up, right down).
            set_motors_with_floor(base + correction, base - correction);
            action = "PD follow";
        }
        else {
            // ----- Line lost: turn back toward where it last was -----
            // Driving wheel gets full BASE_SPEED so it can't stall while searching.
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
