#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Degree scale: 0 = 670 us, 180 = 2470 us (10 us per degree).
// Actual pulses are limited to 800-2200 us (13 to 153).
const int US_MIN = 670;
const int US_MAX = 2470;
const float ANGLE_MIN = 13.0;
const float ANGLE_MAX = 153.0;

// ---------- channels (leg order b1, b2, b3, b4) ----------
const int HIP_CH[4]   = {1, 4, 8, 12};
const int JOINT_CH[4] = {2, 5, 9, 13};
const int FOOT_CH[4]  = {3, 6, 10, 14};

// ---------- geometry (mm, degrees) ----------
const float L1   = 118.25;
const float L2   = 81.81;
const float R    = 76.98;
const float COXA = 55.0;
const float LEG_ANGLE[4] = {72, 288, 144, 216};

// Standing pose: feet form a rectangle around the body center
const float STAND_H = 180.0;                      // mm, joint shaft down to foot tip
const float STAND_R = 40.0;                       // mm, foot tip out from joint shaft
const float STAND_SWING[4] = {-30, 30, -30, 30};  // hip turn at standing, degrees

// ---------- calibration (horns re-seated) ----------
const float HIP_SIGN[4] = {1, 1, 1, 1};           // flip one to -1 if that hip turns the wrong way
const float JOINT_SIGN  = -1;
const float FOOT_SIGN   = -1;
const float HIP_OFF[4]   = {90, 90, 90, 90};      // leg straight out
const float JOINT_OFF[4] = {60, 70, 60, 60};      // thigh horizontal
const float FOOT_OFF[4]  = {145, 145, 145, 145};  // leg straight

// ---------- safety limits ----------
const float MAX_OFFSET   = 80.0;
const float MAX_YAW      = 60.0;
const float MIN_HIP_DIST = 5.0;
const unsigned long MOVE_MS = 600;

// ---------- balance test settings ----------
const float TEST_LIFT  = 35.0;       // mm the test leg lifts
const float SHIFT_STEP = 5.0;        // mm added each try
const float SHIFT_MAX  = 60.0;       // biggest shift tried
const float SAG_COMP   = 10.0;       // mm the two neighbor legs push down while a leg is lifted
const unsigned long LIFT_MS = 1500;  // slower lift so the weight transfers gently

// ---------- types ----------
struct Vec3 { float x, y, z; };
struct A { float thetaK, thetaT; };
struct Servos { float hip, joint, foot; bool ok; const char* why; };

// ---------- state ----------
bool standing = false;
Vec3 offsetNow[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
Servos cur[4];
float foundShift[4] = {-1, -1, -1, -1};

// ---------- IK ----------
Vec3 hipPosition(float R, float V_deg) {
  float V = radians(V_deg);
  return { R * cos(V), R * sin(V), 0.0 };
}

Vec3 vectorToFoot(Vec3 P, Vec3 hip) {
  return { P.x - hip.x, P.y - hip.y, P.z - hip.z };
}

float hipYaw(Vec3 v, float V_deg) {
  float r = sqrt(v.x * v.x + v.y * v.y);
  if (r < 0.001) return 0;
  float yaw = degrees(atan2(v.y, v.x)) - V_deg;
  while (yaw > 180)  yaw -= 360;
  while (yaw < -180) yaw += 360;
  return yaw;
}

bool reachCheck(float r, float y) {
  float d = sqrt(r * r + y * y);
  return d >= fabs(L1 - L2) && d <= L1 + L2;
}

A planarSolve(float r, float y) {
  A out;
  float d = sqrt(r * r + y * y);
  float c = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  c = constrain(c, -1.0, 1.0);
  out.thetaK = acos(c);
  out.thetaT = atan2(y, r) + atan2(L2 * sin(out.thetaK), L1 + L2 * cos(out.thetaK));
  return out;
}

bool inRange(float a) { return a >= ANGLE_MIN && a <= ANGLE_MAX; }

Servos legIK(int i, Vec3 P) {
  Servos s;
  s.ok = true;
  s.why = "";

  Vec3 h = hipPosition(R, LEG_ANGLE[i]);
  Vec3 v = vectorToFoot(P, h);
  float rh = sqrt(v.x * v.x + v.y * v.y);

  if (rh < MIN_HIP_DIST) { s.ok = false; s.why = "foot too close to hip axis"; }

  float yaw = hipYaw(v, LEG_ANGLE[i]);
  float r = rh - COXA;
  float y = v.z;

  if (s.ok && !reachCheck(r, y)) { s.ok = false; s.why = "out of reach"; }

  A a = planarSolve(r, y);
  s.hip   = HIP_OFF[i]   + HIP_SIGN[i] * yaw;
  s.joint = JOINT_OFF[i] + JOINT_SIGN  * degrees(a.thetaT);
  s.foot  = FOOT_OFF[i]  + FOOT_SIGN   * degrees(a.thetaK);

  if (s.ok && fabs(yaw) > MAX_YAW) { s.ok = false; s.why = "hip turn too big"; }
  if (s.ok && !(inRange(s.hip) && inRange(s.joint) && inRange(s.foot))) {
    s.ok = false; s.why = "servo past 13-153";
  }
  return s;
}

// Standing foot position: hip turned by STAND_SWING, foot COXA + STAND_R out from the hip
Vec3 standTarget(int i) {
  Vec3 h = hipPosition(R, LEG_ANGLE[i]);
  float a = radians(LEG_ANGLE[i] + STAND_SWING[i]);
  float out = COXA + STAND_R;
  return { h.x + out * cos(a), h.y + out * sin(a), -STAND_H };
}

Vec3 addVec(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }

// ---------- servo output ----------
int angleToUs(float a) {
  a = constrain(a, ANGLE_MIN, ANGLE_MAX);
  return US_MIN + (int)(a / 180.0 * (US_MAX - US_MIN));
}

void writeAngle(int ch, float a) {
  pwm.writeMicroseconds(ch, angleToUs(a));
}

void writeLeg(int i, float hip, float joint, float foot) {
  writeAngle(HIP_CH[i], hip);
  writeAngle(JOINT_CH[i], joint);
  writeAngle(FOOT_CH[i], foot);
}

void rampTo(Servos target[4], bool sel[4], unsigned long ms) {
  Servos from[4];
  for (int i = 0; i < 4; i++) from[i] = cur[i];
  unsigned long start = millis();
  while (true) {
    float f = (millis() - start) / (float)ms;
    if (f > 1) f = 1;
    for (int i = 0; i < 4; i++) {
      if (!sel[i]) continue;
      writeLeg(i,
        from[i].hip   + (target[i].hip   - from[i].hip)   * f,
        from[i].joint + (target[i].joint - from[i].joint) * f,
        from[i].foot  + (target[i].foot  - from[i].foot)  * f);
    }
    if (f >= 1) break;
    delay(20);
  }
  for (int i = 0; i < 4; i++) if (sel[i]) cur[i] = target[i];
}

// ---------- printing ----------
void printLeg(int i, Servos s) {
  Serial.print("b"); Serial.print(i + 1);
  Serial.print("  hip ");   Serial.print(s.hip, 1);
  Serial.print("  joint "); Serial.print(s.joint, 1);
  Serial.print("  foot ");  Serial.print(s.foot, 1);
  if (s.ok) Serial.println("  OK");
  else { Serial.print("  FAIL: "); Serial.println(s.why); }
}

void printState() {
  for (int i = 0; i < 4; i++) {
    Serial.print("b"); Serial.print(i + 1);
    Serial.print("  offset (");
    Serial.print(offsetNow[i].x, 1); Serial.print(", ");
    Serial.print(offsetNow[i].y, 1); Serial.print(", ");
    Serial.print(offsetNow[i].z, 1); Serial.print(")  ");
    printLeg(i, cur[i]);
  }
}

// ---------- moving all feet ----------
bool poseOk(Vec3 off[4], Servos target[4]) {
  bool ok = true;
  for (int i = 0; i < 4; i++) {
    target[i] = legIK(i, addVec(standTarget(i), off[i]));
    if (!target[i].ok) { printLeg(i, target[i]); ok = false; }
  }
  return ok;
}

bool moveFeet(Vec3 off[4], unsigned long ms) {
  Servos target[4];
  if (!poseOk(off, target)) return false;
  bool sel[4] = {true, true, true, true};
  rampTo(target, sel, ms);
  for (int i = 0; i < 4; i++) offsetNow[i] = off[i];
  return true;
}

// ---------- balance test ----------
int diagonalPartner(int leg) {
  return (leg == 0) ? 3 : (leg == 3) ? 0 : (leg == 1) ? 2 : 1;
}

// Direction the BODY should move before lifting a leg:
// perpendicular to the line between the two side legs, toward the diagonal partner.
Vec3 bodyShiftDir(int lift) {
  int partner = diagonalPartner(lift);
  int a = -1, b = -1;
  for (int k = 0; k < 4; k++) {
    if (k == lift || k == partner) continue;
    if (a < 0) a = k; else b = k;
  }
  Vec3 A_ = standTarget(a), B_ = standTarget(b), P_ = standTarget(partner);
  float ex = B_.x - A_.x, ey = B_.y - A_.y;
  float L = sqrt(ex * ex + ey * ey);
  float nx = -ey / L, ny = ex / L;
  if ((P_.x - A_.x) * nx + (P_.y - A_.y) * ny < 0) { nx = -nx; ny = -ny; }
  return { nx, ny, 0 };
}

char waitKey() {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'y' || c == 'n' || c == 'x') return c;
    }
  }
}

void balanceTest(int leg) {
  if (!standing) { Serial.println(F("Send g first to stand.")); return; }

  Vec3 n = bodyShiftDir(leg);
  int partner = diagonalPartner(leg);
  Vec3 off[4], liftOff[4];
  float result = -1;

  Serial.print(F("Balance test, leg b")); Serial.println(leg + 1);

  for (float s = 0; s <= SHIFT_MAX; s += SHIFT_STEP) {
    Vec3 base = { -n.x * s, -n.y * s, 0 };   // feet move opposite to body
    for (int i = 0; i < 4; i++) { off[i] = base; liftOff[i] = base; }
    liftOff[leg].z += TEST_LIFT;
    for (int i = 0; i < 4; i++) {
      if (i != leg && i != partner) liftOff[i].z -= SAG_COMP;  // neighbors push down
    }

    Servos tmp[4];
    if (!poseOk(off, tmp) || !poseOk(liftOff, tmp)) {
      Serial.print(F("Shift ")); Serial.print(s, 0);
      Serial.println(F(" mm is past the servo limits. Stopping."));
      break;
    }

    moveFeet(off, 1000);          // slide body
    moveFeet(liftOff, LIFT_MS);   // lift test leg, neighbors push down

    Serial.print(F("Shift ")); Serial.print(s, 0);
    Serial.print(F(" mm, feet offset ("));
    Serial.print(base.x, 1); Serial.print(", "); Serial.print(base.y, 1);
    Serial.println(F(")."));
    Serial.println(F("Stayed up?  y = yes   n = no, try more   x = stop"));

    char c = waitKey();
    moveFeet(off, LIFT_MS);       // put the leg back down

    if (c == 'y') { result = s; break; }
    if (c == 'x') break;
  }

  for (int i = 0; i < 4; i++) off[i] = {0, 0, 0};
  moveFeet(off, 1000);            // back to center

  if (result >= 0) {
    foundShift[leg] = result;
    Serial.print(F("RESULT b")); Serial.print(leg + 1);
    Serial.print(F(": balanced with ")); Serial.print(result, 0);
    Serial.print(F(" mm shift, feet offset ("));
    Serial.print(-n.x * result, 1); Serial.print(", ");
    Serial.print(-n.y * result, 1); Serial.println(")");
  } else {
    Serial.print(F("RESULT b")); Serial.print(leg + 1);
    Serial.println(F(": no balanced shift found."));
  }
}

void printResults() {
  Serial.println(F("Balance shifts found (mm, -1 = not tested or none):"));
  for (int i = 0; i < 4; i++) {
    Serial.print("b"); Serial.print(i + 1); Serial.print("  ");
    Serial.println(foundShift[i], 0);
  }
}

// ---------- parsing ----------
int parseNumbers(String s, float out[], int maxN) {
  s.replace("(", " ");
  s.replace(")", " ");
  s.replace(",", " ");
  int n = 0, i = 0, len = s.length();
  while (i < len && n < maxN) {
    while (i < len && s.charAt(i) == ' ') i++;
    if (i >= len) break;
    int j = i;
    while (j < len && s.charAt(j) != ' ') j++;
    out[n++] = s.substring(i, j).toFloat();
    i = j;
  }
  return n;
}

// ---------- commands ----------
void standUp() {
  Servos target[4];
  bool sel[4] = {true, true, true, true};
  for (int i = 0; i < 4; i++) {
    target[i] = legIK(i, standTarget(i));
    if (!target[i].ok) { Serial.print("Stand failed: "); printLeg(i, target[i]); return; }
  }

  if (standing) {
    rampTo(target, sel, MOVE_MS);
  } else {
    for (int i = 0; i < 4; i++) {
      cur[i] = {90, 90, 90, true, ""};
      writeLeg(i, 90, 90, 90);
      bool one[4] = {false, false, false, false};
      one[i] = true;
      rampTo(target, one, 2000);
      delay(800);
    }
    standing = true;
  }
  for (int i = 0; i < 4; i++) offsetNow[i] = {0, 0, 0};
  Serial.println(F("Standing."));
}

void moveCommand(String line) {
  if (!standing) { Serial.println(F("Send g first to stand.")); return; }

  bool sel[4] = {false, false, false, false};
  char first = line.charAt(0);
  if (first == 'a') {
    for (int i = 0; i < 4; i++) sel[i] = true;
  } else if (first >= '1' && first <= '4') {
    sel[first - '1'] = true;
  } else {
    Serial.println(F("Use: g | p | r | t1-t4 | 1 (x,y,z) | a (x,y,z)"));
    return;
  }

  float n[3];
  if (parseNumbers(line.substring(1), n, 3) != 3) {
    Serial.println(F("Need 3 numbers, e.g. 1 (0,0,20)"));
    return;
  }
  Vec3 off = {n[0], n[1], n[2]};

  if (fabs(off.x) > MAX_OFFSET || fabs(off.y) > MAX_OFFSET || fabs(off.z) > MAX_OFFSET) {
    Serial.println(F("Rejected: offset bigger than MAX_OFFSET. Not moving."));
    return;
  }

  Vec3 newOff[4];
  for (int i = 0; i < 4; i++) newOff[i] = sel[i] ? off : offsetNow[i];

  if (moveFeet(newOff, MOVE_MS)) Serial.println(F("Moved."));
  else Serial.println(F("Rejected. Not moving."));
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line == "g") { standUp(); return; }
  if (line == "p") { printState(); return; }
  if (line == "r") { printResults(); return; }
  if (line.length() == 2 && line.charAt(0) == 't' &&
      line.charAt(1) >= '1' && line.charAt(1) <= '4') {
    balanceTest(line.charAt(1) - '1');
    return;
  }
  moveCommand(line);
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("boot"));
  pwm.begin();
  pwm.setOscillatorFrequency(27488192);
  pwm.setPWMFreq(50);
  delay(10);

  Serial.println(F("IK check, standing pose:"));
  for (int i = 0; i < 4; i++) printLeg(i, legIK(i, standTarget(i)));
  Serial.println(F("Commands: g | p | r | t1-t4 | 1 (x,y,z) | a (x,y,z)"));
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}