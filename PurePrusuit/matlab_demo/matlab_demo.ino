/*
  matlab_demo.ino

  Purpose:
    Arduino-side "setup-only" demo inspired by the provided MATLAB Pure Pursuit script.
    - Defines the same waypoint path (converted to mm)
    - Configures Pure Pursuit lookahead and controller limits
    - DOES NOT simulate robot motion

  How to use:
    1) Open this sketch in Arduino IDE.
    2) Ensure the PreciseMovement/PreMo library is available (it provides PurePursuit).
    3) Run and open Serial Monitor @ 115200 baud.
    4) Send pose updates from your real localization source, or type them manually:
         POSE   <x_m>  <y_m>  <theta_rad>
         POSEMM <x_mm> <y_mm> <theta_rad>

    The sketch will compute and print (v, omega) commands based on the current pose
    and the current lookahead goal point on the path.
*/

#include <Arduino.h>
#include <PurePursuit.h>

// -----------------------------
// 1. Define Waypoints and Setup
// -----------------------------

static constexpr float M_TO_MM = 1000.0f;

// MATLAB path waypoints (meters) converted to millimeters for this library/demo.
static constexpr int PATH_LEN = 26;
static float pathX[PATH_LEN] = {
  1.00f * M_TO_MM,
  1.00f * M_TO_MM,
  1.00f * M_TO_MM,
  1.00f * M_TO_MM,
  1.00f * M_TO_MM,
  2.00f * M_TO_MM,
  3.00f * M_TO_MM,
  4.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  4.00f * M_TO_MM,
  3.00f * M_TO_MM,
  2.00f * M_TO_MM,
  2.00f * M_TO_MM,
  2.00f * M_TO_MM,
  2.00f * M_TO_MM,
  3.00f * M_TO_MM,
  4.00f * M_TO_MM,
  4.00f * M_TO_MM,
  4.00f * M_TO_MM,
  3.00f * M_TO_MM,
  3.00f * M_TO_MM,
  3.00f * M_TO_MM
};

static float pathY[PATH_LEN] = {
  1.00f * M_TO_MM,
  2.00f * M_TO_MM,
  3.00f * M_TO_MM,
  4.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  5.00f * M_TO_MM,
  4.00f * M_TO_MM,
  3.00f * M_TO_MM,
  2.00f * M_TO_MM,
  1.00f * M_TO_MM,
  1.00f * M_TO_MM,
  1.00f * M_TO_MM,
  1.00f * M_TO_MM,
  2.00f * M_TO_MM,
  3.00f * M_TO_MM,
  4.00f * M_TO_MM,
  4.00f * M_TO_MM,
  4.00f * M_TO_MM,
  3.00f * M_TO_MM,
  2.00f * M_TO_MM,
  2.00f * M_TO_MM,
  3.00f * M_TO_MM,
  3.00f * M_TO_MM
};

// Robot pose (mm, mm, rad). This should be updated by your real localization.
static double xPos_mm = pathX[0];
static double yPos_mm = pathY[0];

// NOTE: Heading convention can vary by system. The underlying library applies an internal
// offset (`phi = heading - PI/2`). If your turning direction looks off, try sending
// `theta_rad + PI/2` as your heading.
static double heading_rad = 0.0;

// -----------------------------
// 2. Controller Configuration
// -----------------------------

// MATLAB parameters:
//   DesiredLinearVelocity = 0.9 m/s
//   MaxAngularVelocity    = 2.0 rad/s
//   LookaheadDistance     = 0.1 m
static constexpr float DESIRED_LINEAR_VEL_MPS = 0.9f;
static constexpr float MAX_ANGULAR_VEL_RADPS  = 2.0f;
static constexpr float LOOKAHEAD_MM           = 0.1f * M_TO_MM;

// PurePursuit compute interval (ms). This only rate-limits the internal goal-point search.
static constexpr unsigned long COMPUTE_INTERVAL_MS = 50;

PurePursuit pp(&xPos_mm, &yPos_mm, &heading_rad, LOOKAHEAD_MM, COMPUTE_INTERVAL_MS);

// Optional: Catmull-Rom interpolation step (mm). Default in library is 20 mm.
static constexpr float INTERPOLATION_STEP_MM = PurePursuit::DEFAULT_INTERPOLATION_STEP;

// -----------------------------
// 3. Runtime (no simulation)
// -----------------------------

static constexpr unsigned long PRINT_INTERVAL_MS = 100;
static unsigned long lastPrintMs = 0;

static float wrapAnglePi(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

// Parse:
//   "POSE   x_m  y_m  theta_rad"
//   "POSEMM x_mm y_mm theta_rad"
static bool tryParsePoseLine(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return false;

  bool isMm = false;
  if (s.startsWith("POSEMM")) {
    isMm = true;
    s.remove(0, 6);
  } else if (s.startsWith("POSE")) {
    isMm = false;
    s.remove(0, 4);
  } else {
    return false;
  }

  s.trim();

  // Tokenize into 3 numbers.
  float a = NAN, b = NAN, c = NAN;

  char buf[96];
  s.toCharArray(buf, sizeof(buf));
  char* saveptr = nullptr;
  char* tok = strtok_r(buf, " ,\t", &saveptr);
  if (!tok) return false;
  a = (float)atof(tok);
  tok = strtok_r(nullptr, " ,\t", &saveptr);
  if (!tok) return false;
  b = (float)atof(tok);
  tok = strtok_r(nullptr, " ,\t", &saveptr);
  if (!tok) return false;
  c = (float)atof(tok);

  // Basic validation (portable across Arduino cores).
  if (isnan(a) || isnan(b) || isnan(c)) return false;

  if (isMm) {
    xPos_mm = a;
    yPos_mm = b;
  } else {
    xPos_mm = (double)(a * M_TO_MM);
    yPos_mm = (double)(b * M_TO_MM);
  }
  heading_rad = (double)c;
  return true;
}

static void printHelp() {
  Serial.println();
  Serial.println(F("matlab_demo (no simulation)"));
  Serial.println(F("Send pose updates as:"));
  Serial.println(F("  POSE   <x_m>  <y_m>  <theta_rad>"));
  Serial.println(F("  POSEMM <x_mm> <y_mm> <theta_rad>"));
  Serial.println(F("Example: POSE 1.2 3.4 0.0"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pp.setPath(pathX, pathY, PATH_LEN);
  pp.setInterpolationStep(INTERPOLATION_STEP_MM);
  pp.start();

  printHelp();
}

void loop() {
  // Non-blocking Serial line read.
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    if (tryParsePoseLine(line)) {
      // Recompute immediately on new pose.
      pp.compute();
    } else {
      String s = line;
      s.trim();
      if (s.equalsIgnoreCase("HELP")) {
        printHelp();
      }
    }
  }

  // Update internal goal-point selection at the library's rate limit.
  pp.compute();

  // Print control output periodically.
  const unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = now;

  // Goal point chosen by the library (mm).
  const float goalX_mm = pp.getGoalX();
  const float goalY_mm = pp.getGoalY();

  // Pure Pursuit control law (MATLAB-style):
  //   alpha = angle_to_goal - heading
  //   curvature = 2*sin(alpha) / lookahead
  //   omega = v * curvature
  const float dx = goalX_mm - (float)xPos_mm;
  const float dy = goalY_mm - (float)yPos_mm;
  const float angleToGoal = atan2f(dy, dx);
  const float alpha = wrapAnglePi(angleToGoal - (float)heading_rad);

  const float curvature_per_mm = (2.0f * sinf(alpha)) / LOOKAHEAD_MM; // 1/mm
  const float v_mmps = DESIRED_LINEAR_VEL_MPS * M_TO_MM;              // mm/s
  float omega_radps = v_mmps * curvature_per_mm;                      // rad/s

  if (omega_radps > MAX_ANGULAR_VEL_RADPS) omega_radps = MAX_ANGULAR_VEL_RADPS;
  if (omega_radps < -MAX_ANGULAR_VEL_RADPS) omega_radps = -MAX_ANGULAR_VEL_RADPS;

  const bool stop = pp.checkStop();
  const float v_cmd_mps = stop ? 0.0f : DESIRED_LINEAR_VEL_MPS;
  const float omega_cmd = stop ? 0.0f : omega_radps;

  Serial.print(F("pose_mm=("));
  Serial.print((float)xPos_mm, 1);
  Serial.print(F(","));
  Serial.print((float)yPos_mm, 1);
  Serial.print(F(","));
  Serial.print((float)heading_rad, 3);
  Serial.print(F(") goal_mm=("));
  Serial.print(goalX_mm, 1);
  Serial.print(F(","));
  Serial.print(goalY_mm, 1);
  Serial.print(F(") v="));
  Serial.print(v_cmd_mps, 3);
  Serial.print(F(" m/s omega="));
  Serial.print(omega_cmd, 3);
  Serial.print(F(" rad/s"));
  if (stop) Serial.print(F(" [STOP]"));
  Serial.println();
}

