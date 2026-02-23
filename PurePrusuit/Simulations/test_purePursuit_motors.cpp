/**
 * Test harness: (x, y, heading) -> (leftMotor, rightMotor)
 * Tests the full pure pursuit pipeline from RW_Tag_PF_v2.ino
 *
 * Compile & run:
 *   g++ -std=c++14 -o test_purePursuit_motors test_purePursuit_motors.cpp
 *   ./test_purePursuit_motors
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================= Constants (from RW_Tag_PF_v2.ino) =======================

static constexpr float look_ahead     = 100.0f;  // cm
static constexpr float velocity       = 25.0f;   // cm/s
static constexpr float wheelRadius    = 15.0f;   // cm
static constexpr float trackWidth     = 86.0f;   // cm
static constexpr int   waypoint_radius = 25;     // cm
static constexpr int   Encoder_CPR    = 300;

// ======================= Types & state =======================

struct Waypoint { float wp_x; float wp_y; };
struct GoalResult { float gx; float gy; bool found; };

struct MotorOutput {
    float leftMotor;   // rad/s
    float rightMotor;  // rad/s
    int32_t leftQPPS;
    int32_t rightQPPS;
    bool valid;        // false if goal not found or L_d < 1
};

static constexpr int MAX_PATH = 16;
static Waypoint path[MAX_PATH];
static int      PATH_LENGTH;
static int      pathSegIdx;
static double   currentX_global;
static double   currentY_global;
static double   global_azimuth;

// ======================= Pure pursuit functions (from .ino) =======================

static float wrapAnglePi(float a) {
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static int32_t radPerSecToQPPS(float radPerSec) {
    return (int32_t)(radPerSec * Encoder_CPR / (2.0f * (float)M_PI));
}

static void advancePathSegment() {
    if (pathSegIdx < PATH_LENGTH - 1) pathSegIdx++;
}

static void AdvancePathSegment() {
    if (pathSegIdx >= PATH_LENGTH - 1) return;
    int nextWaypoint = pathSegIdx + 1;
    float deltaX = path[nextWaypoint].wp_x - (float)currentX_global;
    float deltaY = path[nextWaypoint].wp_y - (float)currentY_global;
    float distanceToNextWaypoint_sq = deltaX*deltaX + deltaY*deltaY;
    if (distanceToNextWaypoint_sq <= (float)(waypoint_radius * waypoint_radius)) {
        advancePathSegment();
    }
}

static constexpr int MAX_WHILE_ITERS = 50;

GoalResult findLookaheadGoal() {
    GoalResult result;
    result.found = false;
    float Lsq = look_ahead * look_ahead;
    bool miss_wp = true;
    int guard = 0;

    while (miss_wp) {
        if (++guard > MAX_WHILE_ITERS) return result;
        for (int seg = pathSegIdx; seg < PATH_LENGTH - 1; seg++) {
            float dsx = path[seg + 1].wp_x - path[seg].wp_x;
            float dsy = path[seg + 1].wp_y - path[seg].wp_y;
            float fx = path[seg].wp_x - (float)currentX_global;
            float fy = path[seg].wp_y - (float)currentY_global;

            float qa = dsx*dsx + dsy*dsy;
            float qb = 2.0f * (fx*dsx + fy*dsy);
            float qc = (fx*fx + fy*fy) - Lsq;
            float discriminant = qb*qb - 4.0f*qa*qc;

            if (discriminant < 0.0f) {
                miss_wp = false;
            } else {
                float sqrtDisc = sqrtf(discriminant);
                float t1 = (-qb - sqrtDisc) / (2.0f * qa);
                float t2 = (-qb + sqrtDisc) / (2.0f * qa);
                float bestT = -1.0f;
                if (t2 >= 0.0f && t2 <= 1.0f) bestT = t2;
                else if (t1 >= 0.0f && t1 <= 1.0f) bestT = t1;

                if (bestT >= 0.0f) {
                    result.gx = path[seg].wp_x + bestT * dsx;
                    result.gy = path[seg].wp_y + bestT * dsy;
                    result.found = true;
                    return result;
                }
                miss_wp = false;
            }
        }
    }
    return result;
}

// ======================= Main pipeline: (x, y, heading) -> motors =======================

MotorOutput purePursuitStep(float x, float y, float headingRad, int segIdx) {
    currentX_global = x;
    currentY_global = y;
    global_azimuth = headingRad;
    pathSegIdx = segIdx;

    AdvancePathSegment();
    GoalResult goal = findLookaheadGoal();

    MotorOutput out;
    out.leftMotor = 0; out.rightMotor = 0; out.leftQPPS = 0; out.rightQPPS = 0; out.valid = false;
    if (!goal.found) return out;

    float delta_x = goal.gx - (float)currentX_global;
    float delta_y = goal.gy - (float)currentY_global;
    float L_d2 = delta_x*delta_x + delta_y*delta_y;
    float L_d = sqrtf(L_d2);

    if (L_d < 1.0f) return out;

    float angleToGoal = atan2f(delta_y, delta_x);
    float alpha = wrapAnglePi(angleToGoal - (float)global_azimuth);
    float K = 2.0f * sinf(alpha) / L_d;
    float omega = K * velocity;

    out.leftMotor  = (velocity - omega * trackWidth / 2.0f) / wheelRadius;
    out.rightMotor = (velocity + omega * trackWidth / 2.0f) / wheelRadius;
    out.leftQPPS   = radPerSecToQPPS(out.leftMotor);
    out.rightQPPS  = radPerSecToQPPS(out.rightMotor);
    out.valid      = true;
    return out;
}

// ======================= Test harness =======================

static void setPath(const Waypoint* wp, int n) {
    PATH_LENGTH = n;
    for (int i = 0; i < n; i++) path[i] = wp[i];
}

static int passed = 0, failed = 0;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  PASS: %s\n", msg); passed++; } \
    else { printf("  FAIL: %s\n", msg); failed++; } \
} while(0)

#define DEG2RAD(d) ((d) * (float)M_PI / 180.0f)

int main() {
    // Updated path: rectangle (200,200)->(200,2441)->(729,2441)->(729,200)->(1200,200)
    Waypoint defaultPath[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(defaultPath, 5);

    printf("====== Pure Pursuit Motor Tests (updated waypoints) ======\n");
    printf("Path: (200,200)->(200,2441)->(729,2441)->(729,200)->(1200,200)\n\n");

    // --- Test 1: On first segment (vertical up), heading north ---
    {
        printf("Test 1: Robot at (200,800), heading 90 deg (north), on segment 0\n");
        MotorOutput m = purePursuitStep(200.0f, 800.0f, DEG2RAD(90), 0);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s, leftQPPS=%ld, rightQPPS=%ld\n",
               m.leftMotor, m.rightMotor, (long)m.leftQPPS, (long)m.rightQPPS);
        CHECK(m.leftMotor > 0 && m.rightMotor > 0, "both motors positive (forward)");
        CHECK(fabsf(m.leftMotor - m.rightMotor) < 0.3f, "nearly straight (vertical path)");
        printf("\n");
    }

    // --- Test 2: On first segment, heading east (need to turn left toward north) ---
    {
        printf("Test 2: Robot at (200,1500), heading 0 deg (east), path goes north\n");
        MotorOutput m = purePursuitStep(200.0f, 1500.0f, DEG2RAD(0), 0);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        CHECK(m.rightMotor > m.leftMotor, "turn left toward goal (right motor faster)");
        printf("\n");
    }

    // --- Test 3: On segment 1 (horizontal east), heading east ---
    {
        printf("Test 3: Robot at (400,2441), heading 0 deg, on horizontal segment 1\n");
        MotorOutput m = purePursuitStep(400.0f, 2441.0f, DEG2RAD(0), 1);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        CHECK(fabsf(m.leftMotor - m.rightMotor) < 0.3f, "nearly straight (horizontal path)");
        printf("\n");
    }

    // --- Test 4: At start waypoint, heading north (aligned) ---
    {
        printf("Test 4: Robot at (200,200), heading 90 deg (along segment 0)\n");
        MotorOutput m = purePursuitStep(200.0f, 200.0f, DEG2RAD(90), 0);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        CHECK(fabsf(m.leftMotor - m.rightMotor) < 0.3f, "nearly equal (straight)");
        printf("\n");
    }

    // --- Test 5: Sharp turn at corner (wp1: from north to east) ---
    {
        printf("Test 5: Robot at (200,2400), heading 90 deg, approaching corner to turn east\n");
        MotorOutput m = purePursuitStep(200.0f, 2400.0f, DEG2RAD(90), 0);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        printf("\n");
    }

    // --- Test 6: On segment 2 (vertical down), heading south ---
    {
        printf("Test 6: Robot at (729,1500), heading 270 deg (south), on segment 2\n");
        MotorOutput m = purePursuitStep(729.0f, 1500.0f, DEG2RAD(270), 2);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        CHECK(fabsf(m.leftMotor - m.rightMotor) < 0.3f, "nearly straight on vertical");
        printf("\n");
    }

    // --- Test 7: Offset right of segment 0 (x=200), need to turn left to rejoin ---
    {
        printf("Test 7: Robot at (250,1500), heading 90 deg, path at x=200 (robot east of path)\n");
        MotorOutput m = purePursuitStep(250.0f, 1500.0f, DEG2RAD(90), 0);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        CHECK(m.rightMotor > m.leftMotor, "turn left to rejoin path (goal west)");
        printf("\n");
    }

    // --- Test 8: Offset left of segment 0, need to turn right to rejoin ---
    {
        printf("Test 8: Robot at (150,1500), heading 90 deg, path at x=200 (robot west of path)\n");
        MotorOutput m = purePursuitStep(150.0f, 1500.0f, DEG2RAD(90), 0);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        CHECK(m.leftMotor > m.rightMotor, "turn right to rejoin path (goal east)");
        printf("\n");
    }

    // --- Test 9: Near waypoint 1, about to advance to segment 1 ---
    {
        printf("Test 9: Robot at (205,2430), within waypoint radius of wp1\n");
        MotorOutput m = purePursuitStep(205.0f, 2430.0f, DEG2RAD(90), 0);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s, pathSegIdx after=%d\n",
               m.leftMotor, m.rightMotor, pathSegIdx);
        printf("\n");
    }

    // --- Test 10: Circle too small (far from path) -> invalid ---
    {
        printf("Test 10: Robot at (0,0), heading 0 deg, circle cannot reach path\n");
        MotorOutput m = purePursuitStep(0.0f, 0.0f, DEG2RAD(0), 0);
        CHECK(!m.valid, "output invalid (goal not found)");
        printf("\n");
    }

    // --- Test 11: Last segment (segment 3), approaching finish ---
    {
        printf("Test 11: Robot at (900,200), heading 0 deg, last segment\n");
        MotorOutput m = purePursuitStep(900.0f, 200.0f, DEG2RAD(0), 3);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        printf("\n");
    }

    // --- Test 12: Different headings produce different turn commands ---
    {
        printf("Test 12: Robot at (200,500), heading 90 deg vs 45 deg\n");
        MotorOutput m1 = purePursuitStep(200.0f, 500.0f, DEG2RAD(90), 0);
        MotorOutput m2 = purePursuitStep(200.0f, 500.0f, DEG2RAD(45), 0);
        CHECK(m1.valid && m2.valid, "both valid");
        printf("       heading 90: L=%.3f R=%.3f | heading 45: L=%.3f R=%.3f\n",
               m1.leftMotor, m1.rightMotor, m2.leftMotor, m2.rightMotor);
        CHECK(m1.leftMotor != m2.leftMotor || m1.rightMotor != m2.rightMotor, "different headings yield different outputs");
        printf("\n");
    }

    // --- Test 13: Corner at wp2, transition from east to south ---
    {
        printf("Test 13: Robot at (700,2441), heading 0 deg, approaching corner to turn south\n");
        MotorOutput m = purePursuitStep(700.0f, 2441.0f, DEG2RAD(0), 1);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        printf("\n");
    }

    // --- Test 14: Mid segment 1, slight heading error ---
    {
        printf("Test 14: Robot at (450,2441), heading 5 deg (slightly off east)\n");
        MotorOutput m = purePursuitStep(450.0f, 2441.0f, DEG2RAD(5), 1);
        CHECK(m.valid, "output valid");
        printf("       leftMotor=%.3f rad/s, rightMotor=%.3f rad/s\n", m.leftMotor, m.rightMotor);
        printf("\n");
    }

    // --- Test 15: QPPS range sanity ---
    {
        printf("Test 15: Motor outputs produce reasonable QPPS (|QPPS| < 10000)\n");
        MotorOutput m = purePursuitStep(200.0f, 1000.0f, DEG2RAD(90), 0);
        CHECK(m.valid, "output valid");
        int lim = 10000;
        CHECK(m.leftQPPS > -lim && m.leftQPPS < lim && m.rightQPPS > -lim && m.rightQPPS < lim,
              "QPPS in reasonable range");
        printf("       leftQPPS=%ld, rightQPPS=%ld\n", (long)m.leftQPPS, (long)m.rightQPPS);
        printf("\n");
    }

    printf("====== Results: %d passed, %d failed ======\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
