/**
 * Desktop test harness for findLookaheadGoal()
 * from RW_Tag_PF_v2.ino
 *
 * Compile & run:
 *   g++ -std=c++17 -o test_findLookaheadGoal test_findLookaheadGoal.cpp -lm
 *   ./test_findLookaheadGoal
 *
 * On Windows (MSVC):
 *   cl /EHsc /std:c++17 test_findLookaheadGoal.cpp
 *   test_findLookaheadGoal.exe
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cassert>

// ======================= Extracted types & state =======================

struct Waypoint {
    float wp_x;
    float wp_y;
};

struct GoalResult {
    float gx;
    float gy;
    bool  found;
};

static constexpr int MAX_PATH = 16;

// Mutable test state (mirrors the globals in the .ino)
static Waypoint path[MAX_PATH];
static int      PATH_LENGTH;
static int      pathSegIdx;
static double   currentX_global;
static double   currentY_global;
static float    look_ahead;

// =================== Function under test (verbatim copy) ===================
// NOTE: The original while(miss_wp) loop can spin forever when the discriminant
// is >= 0 but no valid t in [0,1] exists (bestT stays -1).  The test harness
// adds an iteration guard so tests don't hang; the real firmware should be fixed.

static constexpr int MAX_WHILE_ITERS = 50;

GoalResult findLookaheadGoal() {
    GoalResult result;
    result.found = false;

    float Lsq = look_ahead * look_ahead;

    bool miss_wp;
    miss_wp = true;

    int guard = 0;
    while (miss_wp) {
        if (++guard > MAX_WHILE_ITERS) {
            return result;
        }

        for (int seg = pathSegIdx; seg < PATH_LENGTH - 1; seg++) {
            float dsx = path[seg + 1].wp_x - path[seg].wp_x;
            float dsy = path[seg + 1].wp_y - path[seg].wp_y;

            float fx = path[seg].wp_x - (float)currentX_global;
            float fy = path[seg].wp_y - (float)currentY_global;

            float qa = dsx * dsx + dsy * dsy;
            float qb = 2.0f * (fx * dsx + fy * dsy);
            float qc = (fx * fx + fy * fy) - Lsq;

            float discriminant = qb * qb - 4.0f * qa * qc;

            if (discriminant < 0.0f) {
                miss_wp = false;
            } else {
                float sqrtDisc = sqrtf(discriminant);

                float t1 = (-qb - sqrtDisc) / (2.0f * qa);
                float t2 = (-qb + sqrtDisc) / (2.0f * qa);

                float bestT = -1.0f;
                if (t2 >= 0.0f && t2 <= 1.0f) {
                    bestT = t2;
                } else if (t1 >= 0.0f && t1 <= 1.0f) {
                    bestT = t1;
                }

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

// ======================== Test helpers ========================

static void resetState() {
    pathSegIdx      = 0;
    currentX_global = 0.0;
    currentY_global = 0.0;
    look_ahead      = 100.0f;
    PATH_LENGTH     = 0;
    memset(path, 0, sizeof(path));
}

static void setPath(const Waypoint* wp, int n) {
    PATH_LENGTH = n;
    for (int i = 0; i < n; i++) path[i] = wp[i];
}

static float distToGoal(const GoalResult& g) {
    float dx = g.gx - (float)currentX_global;
    float dy = g.gy - (float)currentY_global;
    return sqrtf(dx * dx + dy * dy);
}

static bool approxEq(float a, float b, float tol = 1.0f) {
    return fabsf(a - b) <= tol;
}

static int passed = 0;
static int failed = 0;

#define CHECK(cond, msg)                                                \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("  FAIL: %s\n", msg);                                \
            failed++;                                                   \
        } else {                                                        \
            printf("  PASS: %s\n", msg);                                \
            passed++;                                                   \
        }                                                               \
    } while (0)

// ======================== Test cases ========================

/**
 * Test 1: Robot at first waypoint, lookahead circle intersects first segment.
 *
 * Path: (200,200) -> (200,2441) [vertical up]
 * Robot at (200,200), look_ahead=100
 * The goal should lie on segment 0, ~100 cm from the robot.
 */
void test_robotAtFirstWaypoint() {
    printf("Test 1: Robot at first waypoint\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 200.0;
    currentY_global = 200.0;
    look_ahead = 100.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    float d = distToGoal(g);
    CHECK(approxEq(d, look_ahead, 2.0f), "goal is ~look_ahead distance away");
    CHECK(approxEq(g.gx, 200.0f, 2.0f), "goal x on vertical segment (x=200)");
    CHECK(g.gy >= 200.0f && g.gy <= 2441.0f, "goal y within segment y range");
    printf("\n");
}

/**
 * Test 2: Robot midway along first segment.
 *
 * Midpoint of segment 0: (200, 1320.5) [vertical segment 200->2441]
 * look_ahead = 100, should intersect ahead on same segment.
 */
void test_robotMidSegment() {
    printf("Test 2: Robot midway along first segment\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 200.0;
    currentY_global = 1320.5;
    look_ahead = 100.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    float d = distToGoal(g);
    CHECK(approxEq(d, look_ahead, 2.0f), "goal is ~look_ahead distance away");
    printf("\n");
}

/**
 * Test 3: pathSegIdx advanced to segment 1, robot at waypoint 1.
 *
 * Robot at (200, 2441), segment 1: (200,2441)->(729,2441) [horizontal east]
 * Circle should intersect at (300, 2441) [100 cm ahead].
 */
void test_robotAtSecondWaypoint() {
    printf("Test 3: Robot near second waypoint, segment 1 active\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 200.0;
    currentY_global = 2441.0;
    look_ahead = 100.0f;
    pathSegIdx = 1;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    CHECK(approxEq(g.gy, 2441.0f, 2.0f), "goal y on horizontal segment (y=2441)");
    CHECK(approxEq(g.gx, 300.0f, 2.0f), "goal x ~300 (100 cm east)");
    printf("\n");
}

/**
 * Test 4: Robot offset from path, circle still reaches segment.
 *
 * Robot at (250, 2441), 50 cm east of wp1. Segment 1 is horizontal at y=2441.
 * look_ahead = 100, circle should intersect the horizontal segment.
 */
void test_robotOffsetFromPath() {
    printf("Test 4: Robot offset from path, circle still reaches segment\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 250.0;
    currentY_global = 2441.0;
    look_ahead = 100.0f;
    pathSegIdx = 1;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    float d = distToGoal(g);
    CHECK(approxEq(d, look_ahead, 2.0f), "goal is ~look_ahead distance away");
    CHECK(approxEq(g.gy, 2441.0f, 2.0f), "goal y on horizontal segment");
    printf("\n");
}

/**
 * Test 5: Lookahead circle too small to reach any remaining segment.
 *
 * Robot at (0, 0), far from all segments, look_ahead = 10 (tiny).
 * Should trigger the no-intersection fallback / miss_wp guard.
 */
void test_circleDoesNotReachPath() {
    printf("Test 5: Circle too small to reach path\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 0.0;
    currentY_global = 0.0;
    look_ahead = 10.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    // The original code may loop forever here (see bug note); our guard returns found=false
    CHECK(!g.found, "no intersection found (circle too small)");
    printf("\n");
}

/**
 * Test 6: Robot far from path with large lookahead.
 *
 * Robot at (0, 0), look_ahead = 500, should reach the first segment easily.
 */
void test_largeLookaheadFromFarAway() {
    printf("Test 6: Large lookahead from far away\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 0.0;
    currentY_global = 0.0;
    look_ahead = 500.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    float d = distToGoal(g);
    CHECK(approxEq(d, 500.0f, 2.0f), "goal is ~500 cm from robot");
    printf("\n");
}

/**
 * Test 7: Only one segment left (last segment), robot near waypoint 3.
 *
 * pathSegIdx = 3, segment 3->4: (729,200)->(1200,200) [horizontal east]
 * Robot at (729, 200), look_ahead = 100
 * Goal should be at (829, 200).
 */
void test_lastSegment() {
    printf("Test 7: Last segment of the path\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 729.0;
    currentY_global = 200.0;
    look_ahead = 100.0f;
    pathSegIdx = 3;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found on last segment");
    CHECK(approxEq(g.gy, 200.0f, 2.0f), "goal y = 200");
    CHECK(approxEq(g.gx, 829.0f, 2.0f), "goal x ~829 (100 cm east)");
    printf("\n");
}

/**
 * Test 8: Simple two-waypoint horizontal path.
 *
 * Path: (0,0) -> (1000,0). Robot at (100,0), look_ahead=200.
 * Goal should be at (300, 0).
 */
void test_simpleHorizontalPath() {
    printf("Test 8: Simple horizontal path\n");
    resetState();
    Waypoint wp[] = {{0,0},{1000,0}};
    setPath(wp, 2);
    currentX_global = 100.0;
    currentY_global = 0.0;
    look_ahead = 200.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    CHECK(approxEq(g.gx, 300.0f, 1.0f), "goal x = 300");
    CHECK(approxEq(g.gy, 0.0f, 1.0f), "goal y = 0");
    printf("\n");
}

/**
 * Test 9: Robot perpendicular to a horizontal segment.
 *
 * Path: (0,0) -> (1000,0). Robot at (500, 60), look_ahead=100.
 * Circle should intersect the x-axis; goal at (500 + sqrt(100^2 - 60^2), 0)
 * = (500 + 80, 0) = (580, 0).
 */
void test_perpendicularOffset() {
    printf("Test 9: Robot perpendicular to horizontal segment\n");
    resetState();
    Waypoint wp[] = {{0,0},{1000,0}};
    setPath(wp, 2);
    currentX_global = 500.0;
    currentY_global = 60.0;
    look_ahead = 100.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    float expectedX = 500.0f + sqrtf(100.0f*100.0f - 60.0f*60.0f);
    CHECK(approxEq(g.gx, expectedX, 1.0f), "goal x matches expected");
    CHECK(approxEq(g.gy, 0.0f, 1.0f), "goal y on segment (y=0)");
    printf("\n");
}

/**
 * Test 10: Circle tangent to segment (discriminant ~ 0).
 *
 * Path: (0,0) -> (1000,0). Robot at (500, 100), look_ahead = 100.
 * Perpendicular distance = 100 = look_ahead, so tangent.
 * The single tangent point is at (500, 0).
 */
void test_tangentToSegment() {
    printf("Test 10: Circle tangent to segment (discriminant ~ 0)\n");
    resetState();
    Waypoint wp[] = {{0,0},{1000,0}};
    setPath(wp, 2);
    currentX_global = 500.0;
    currentY_global = 100.0;
    look_ahead = 100.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found (tangent)");
    CHECK(approxEq(g.gx, 500.0f, 2.0f), "goal x ~ 500");
    CHECK(approxEq(g.gy, 0.0f, 2.0f), "goal y ~ 0");
    printf("\n");
}

/**
 * Test 11: Circle intersects behind robot on segment but not ahead.
 *
 * Path: (0,0) -> (1000,0). Robot at (950, 0), look_ahead = 200.
 * t2 would put the intersection past the segment end (t > 1).
 * t1 puts it at 750, which is valid.
 */
void test_intersectionNearSegmentEnd() {
    printf("Test 11: Intersection near segment end\n");
    resetState();
    Waypoint wp[] = {{0,0},{1000,0}};
    setPath(wp, 2);
    currentX_global = 950.0;
    currentY_global = 0.0;
    look_ahead = 200.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    // t2 = (950+200)/1000 = 1.15 (out of range), t1 = (950-200)/1000 = 0.75 (valid)
    CHECK(g.found, "intersection found via t1 fallback");
    CHECK(approxEq(g.gx, 750.0f, 2.0f), "goal x ~ 750");
    CHECK(approxEq(g.gy, 0.0f, 1.0f), "goal y = 0");
    printf("\n");
}

/**
 * Test 12: Circle skips current segment, finds intersection on next segment.
 *
 * Path: (0,0) -> (100,0) -> (100,1000). Robot at (50, 0), look_ahead = 80.
 * Segment 0 is only 100 cm long; robot is at midpoint.
 * 80 cm ahead from (50,0) along segment 0 would reach (130,0) - past the end.
 * But segment 1 goes from (100,0) up to (100,1000), so the circle
 * should find an intersection on segment 1.
 */
void test_skipToNextSegment() {
    printf("Test 12: Circle skips to next segment\n");
    resetState();
    Waypoint wp[] = {{0,0},{100,0},{100,1000}};
    setPath(wp, 3);
    currentX_global = 50.0;
    currentY_global = 0.0;
    look_ahead = 80.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found on next segment");
    float d = distToGoal(g);
    CHECK(approxEq(d, 80.0f, 2.0f), "goal is ~80 cm from robot");
    CHECK(approxEq(g.gx, 100.0f, 2.0f), "goal x on vertical segment (x=100)");
    printf("\n");
}

/**
 * Test 13: Diagonal path, robot on segment.
 *
 * Path: (0,0) -> (300,400). Segment length = 500.
 * Robot at (0,0), look_ahead = 100.
 * Goal should be at t = 100/500 = 0.2, i.e. (60, 80).
 */
void test_diagonalPath() {
    printf("Test 13: Diagonal path (3-4-5 triangle)\n");
    resetState();
    Waypoint wp[] = {{0,0},{300,400}};
    setPath(wp, 2);
    currentX_global = 0.0;
    currentY_global = 0.0;
    look_ahead = 100.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found");
    CHECK(approxEq(g.gx, 60.0f, 1.0f), "goal x = 60");
    CHECK(approxEq(g.gy, 80.0f, 1.0f), "goal y = 80");
    printf("\n");
}

/**
 * Test 14: pathSegIdx already at last waypoint (no remaining segments).
 *
 * pathSegIdx = 3 with PATH_LENGTH = 4 means pathSegIdx = PATH_LENGTH-1.
 * The for loop condition `seg < PATH_LENGTH - 1` is immediately false.
 */
void test_pathComplete() {
    printf("Test 14: Path already complete (no segments left)\n");
    resetState();
    Waypoint wp[] = {{200,200},{200,2441},{729,2441},{729,200},{1200,200}};
    setPath(wp, 5);
    currentX_global = 1200.0;
    currentY_global = 200.0;
    look_ahead = 100.0f;
    pathSegIdx = 4;

    GoalResult g = findLookaheadGoal();
    // For loop body never executes; miss_wp stays true -> guard trips
    CHECK(!g.found, "no intersection (path complete, no segments)");
    printf("\n");
}

/**
 * Test 15: Very short segment with robot exactly on it.
 *
 * Path: (100,100) -> (101,100) -> (101,500). Segment 0 is 1 cm long.
 * Robot at (100.5, 100), look_ahead = 50.
 * Too short for seg 0, should find intersection on seg 1.
 */
void test_veryShortSegment() {
    printf("Test 15: Very short first segment, intersection on next\n");
    resetState();
    Waypoint wp[] = {{100,100},{101,100},{101,500}};
    setPath(wp, 3);
    currentX_global = 100.5;
    currentY_global = 100.0;
    look_ahead = 50.0f;
    pathSegIdx = 0;

    GoalResult g = findLookaheadGoal();
    CHECK(g.found, "intersection found (on longer segment)");
    float d = distToGoal(g);
    CHECK(approxEq(d, 50.0f, 2.0f), "goal is ~50 cm from robot");
    printf("\n");
}

// ======================== Main ========================

int main() {
    printf("====== findLookaheadGoal() Test Suite ======\n\n");

    test_robotAtFirstWaypoint();       // 1
    test_robotMidSegment();            // 2
    test_robotAtSecondWaypoint();       // 3
    test_robotOffsetFromPath();         // 4
    test_circleDoesNotReachPath();      // 5
    test_largeLookaheadFromFarAway();   // 6
    test_lastSegment();                 // 7
    test_simpleHorizontalPath();        // 8
    test_perpendicularOffset();         // 9
    test_tangentToSegment();            // 10
    test_intersectionNearSegmentEnd();  // 11
    test_skipToNextSegment();           // 12
    test_diagonalPath();               // 13
    test_pathComplete();                // 14
    test_veryShortSegment();            // 15

    printf("====== Results: %d passed, %d failed ======\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
