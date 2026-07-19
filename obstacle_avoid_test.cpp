#include "mbed.h"

// ============================================================================
//  OBSTACLE AVOIDANCE TEST PROGRAM
//  Nucleo-F303K8 / Mbed
// ----------------------------------------------------------------------------
//  Purpose:
//   A stripped-down test harness for the ultrasonic obstacle avoidance only.
//   The robot drives STRAIGHT forward at all times. It pings the HC-SR04 every
//   SONAR_EVERY loops; when a confirmed close reading is seen it runs the same
//   blocking side-step maneuver used by the full line follower, then returns to
//   driving straight.
//
//   NO line following / PD / IR logic is involved here -- this exists purely to
//   tune and verify the obstacle-avoidance behaviour in isolation.
//
//   Constants and motor / sonar helper functions are kept IDENTICAL to
//   sonartest.cpp so tuning carries straight back to the main program.
//
//   NOTE: This file defines its own main(). In an Mbed build only one main()
//   may exist, so build this file INSTEAD OF sonartest.cpp (exclude the other
//   from the build / move it out of the source tree).
// ============================================================================


// ============================================================================
//  HARDWARE PINS
// ============================================================================
PwmOut left_motor_speed (D11);
PwmOut left_motor_dir   (D12);
PwmOut right_motor_speed(D9);
PwmOut right_motor_dir  (PA_11_ALT0);

DigitalOut trig(D6);   // HC-SR04 TRIG (output)
DigitalIn  echo(D2);   // HC-SR04 ECHO (input)


// ############################################################################
// #                          TUNING CONFIG                                   #
// #        (mirrors sonartest.cpp -- keep the two in sync)                   #
// ############################################################################

// --- Speed ------------------------------------------------------------------
const float SPEED_SCALE             = 0.785f;  // master duty multiplier
const float RIGHT_SPEED_CALIBRATION = 1.0f;    // trim if one wheel runs faster
const float BASE_SPEED              = 0.71f;   // straight-line cruise duty

// --- Motor direction --------------------------------------------------------
const float LEFT_DIR_FORWARD        = 0.0f;
const float RIGHT_DIR_FORWARD       = 0.0f;    // flip to 1.0f if right wheel spins backward

// --- Obstacle avoidance : FRONTAL -------------------------------------------
const float OBSTACLE_DIST_CM = 26.5f;          // trigger distance (cm)
const float TEST_LEFT        = 0.00f;          // left  wheel duty during the pivot
const float TEST_RIGHT       = 0.70f;          // right wheel duty during the pivot
const int   TEST_TIME_MS     = 680;            // pivot hold time = rotation angle
const int   RETURN_TIME_MS   = 1410;           // turn-back time (raise if it stays parallel)
const int   FWD_PAUSE_MS     = 1275;           // forward drive time past the obstacle
const float FWD_DUTY         = 0.70f;          // forward duty during the maneuver

// --- Obstacle avoidance : DIAGONAL ------------------------------------------
const float OBSTACLE_DIST_DIAG_CM = 20.0f;     // switch to diagonal below this (cm)
const int   TEST_TIME_MS_DIAG     = 680;
const int   RETURN_TIME_MS_DIAG   = 1410;
const int   FWD_PAUSE_MS_DIAG     = 1200;

// --- Obstacle detection (sonar) ---------------------------------------------
const float EMERGENCY_DIST_CM = 12.0f;         // one reading below this triggers immediately
const int   SONAR_EVERY       = 2;             // ping every N loops
const float MIN_VALID_CM      = 2.0f;          // ignore readings below this (noise)
const int   OBSTACLE_CONFIRM  = 2;             // consecutive close pings needed to trigger
const int   SONAR_PRINT_EVERY = 10;            // print dist only every Nth ping

// ############################################################################
// #                        END TUNING CONFIG                                 #
// ############################################################################


// ============================================================================
//  MOTOR HELPERS  (identical to sonartest.cpp)
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

void stop_and_settle() {
    set_motors(0.0f, 0.0f);
    ThisThread::sleep_for(std::chrono::milliseconds(150));
}


// ============================================================================
//  SENSOR HELPER  (identical to sonartest.cpp)
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


// ============================================================================
//  OBSTACLE MANEUVERS  (identical to sonartest.cpp)
//  Blocking side-steps: pivot away, drive past, pivot back, then drive
//  forward a bounded amount before handing back to straight driving.
// ============================================================================

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

    // 4) pivot back (mirror of step 2 -> toward the RIGHT)
    set_motors_backward1(FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(RETURN_TIME_MS));
    stop_and_settle();

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

    // 4) pivot back
    set_motors_backward1(FWD_DUTY, -FWD_DUTY);
    ThisThread::sleep_for(std::chrono::milliseconds(RETURN_TIME_MS_DIAG));
    stop_and_settle();

    set_motors(0.0f, 0.0f);
}


// ============================================================================
//  MAIN LOOP  --  drive straight, avoid only when an obstacle is detected
// ============================================================================
int main() {
    left_motor_speed.period(0.02f);
    right_motor_speed.period(0.02f);
    left_motor_dir.period(0.02f);
    right_motor_dir.period(0.02f);

    printf("\n--- OBSTACLE AVOIDANCE TEST : drive straight + avoid ---\n");

    // Ultrasonic sampling state.
    int sonar_counter     = 0;
    int obstacle_hits     = 0;  // confirmation streak (decays, never hard-reset)
    int sonar_print_count = 0;

    while (1) {
        // ----- Obstacle check: ping every SONAR_EVERY loops -----
        if (++sonar_counter >= SONAR_EVERY) {
            sonar_counter = 0;
            float dist = read_distance_cm();

            bool close = (dist > MIN_VALID_CM && dist < OBSTACLE_DIST_CM);
            if (close || ++sonar_print_count >= SONAR_PRINT_EVERY) {
                sonar_print_count = 0;
                printf("dist=%d cm\n", (int)dist);
            }

            if (close) {
                // EMERGENCY band: a valid reading already inside EMERGENCY_DIST_CM
                // triggers immediately, no confirmation streak needed.
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
                    continue;   // re-check distance fresh next loop
                }
            } else {
                // DECAY the streak instead of wiping it, so flickery
                // valid/9999 readings on an angled approach can still confirm.
                if (obstacle_hits > 0) obstacle_hits--;
            }
        }

        // ----- No obstacle: just drive straight -----
        set_motors(BASE_SPEED, BASE_SPEED);

        ThisThread::sleep_for(10ms);
    }
}
