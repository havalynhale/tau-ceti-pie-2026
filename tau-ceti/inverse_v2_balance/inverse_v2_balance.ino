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
const int JOINT_CH[4] = {2, 5, 9, 13};   // change 5 to 7 if b2j is plugged into channel 7
const int FOOT_CH[4]  = {3, 6, 10, 14};

// ---------- geometry (mm, degrees) ----------
const float L1   = 118.25;
const float L2   = 81.81;
const float R    = 76.98;
const float COXA = 55.0;
const float LEG_ANGLE[4] = {72, 288, 144, 216};

// Standing pose: feet form a rectangle around the body center
const float STAND_H = 180.0;
const float STAND_R = 40.0;
const float STAND_SWING[4] = {-30, 30, -30, 30};

// ---------- calibration ----------
const float HIP_SIGN[4] = {1, 1, 1, 1};
const float JOINT_SIGN  = -1;
const float FOOT_SIGN   = -1;
const float HIP_OFF[4]   = {90, 90, 90, 90};
const float JOINT_OFF[4] = {60, 70, 60, 60};
const float FOOT_OFF[4]  = {145, 145, 145, 145};

// ---------- safety limits ----------
const float MAX_YAW      = 60.0;
const float MIN_HIP_DIST = 5.0;
const unsigned long MOVE_MS = 800;

// ---------- walking settings ----------
const float STRIDE      = 30.0;                  // mm the body moves forward per full cycle
const float LIFT        = 35.0;                  // mm the stepping foot lifts
const float SAG_COMP    = 10.0;                  // mm the two neighbor legs push down during a step
const float SHIFT_LEG[4] = {30, 20, 25, 15};     // measured balance shifts (b1, b2, b3, b4)
const float SHIFT_EXTRA = 5.0;                   // extra safety margin on every shift
const int   ORDER[4]    = {2, 0, 3, 1};          // step order: b3, b1, b4, b2
const unsigned long SHIFT_MS = 1000;             // body slide
const unsigned long LIFT_MS  = 1000;             // foot up
const unsigned long SWING_MS = 800;              // foot forward
const unsigned long LOWER_MS = 1000;             // foot down

// ---------- types ----------
struct Vec3 { float x, y, z; };
struct A { float thetaK, thetaT; };
struct Servos { float hip, joint, foot; bool ok; const char* why; };

// ---------- state ----------
bool standing = false;
Vec3 offsetNow[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
Servos cur[4];

float fx[4] = {0, 0, 0, 0};   // forward offset of each foot from its standing spot, mm
int  phase = 0;               // which entry of ORDER steps next
bool inGait = false;          // feet are staggered for walking
bool walking = false;
bool stopRequested = false;

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

// ---------- balance helpers ----------
int diagonalPartner(int leg) {
  return (leg == 0) ? 3 : (leg == 3) ? 0 : (leg == 1) ? 2 : 1;
}

// Direction the BODY moves before lifting a leg
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

// ---------- gait ----------
// Foot offsets for the current fx, a body shift, and an optional lifted leg
void buildOffsets(Vec3 off[4], Vec3 bs, int liftLeg, float liftZ) {
  for (int i = 0; i < 4; i++) {
    off[i].x = fx[i] - bs.x;
    off[i].y = -bs.y;
    off[i].z = 0;
    if (liftLeg >= 0) {
      if (i == liftLeg) off[i].z = liftZ;
      else if (i != diagonalPartner(liftLeg)) off[i].z = -SAG_COMP;
    }
  }
}

// One step: shift body, lift leg L, swing it to newFx, lower it.
// Checks all four poses before moving anything.
bool stepLeg(int L, float newFx) {
  Vec3 n = bodyShiftDir(L);
  float s = SHIFT_LEG[L] + SHIFT_EXTRA;
  Vec3 bs = { n.x * s, n.y * s, 0 };

  float oldFx = fx[L];
  Vec3 pShift[4], pLift[4], pSwing[4], pLower[4];
  buildOffsets(pShift, bs, -1, 0);
  buildOffsets(pLift,  bs, L, LIFT);
  fx[L] = newFx;
  buildOffsets(pSwing, bs, L, LIFT);
  buildOffsets(pLower, bs, -1, 0);
  fx[L] = oldFx;

  Servos tmp[4];
  if (!poseOk(pShift, tmp) || !poseOk(pLift, tmp) ||
      !poseOk(pSwing, tmp) || !poseOk(pLower, tmp)) {
    Serial.print(F("Step for b")); Serial.print(L + 1);
    Serial.println(F(" not reachable. Stopping."));
    return false;
  }

  Serial.print(F("Step b")); Serial.println(L + 1);
  moveFeet(pShift, SHIFT_MS);   // slide body over the other three feet
  moveFeet(pLift,  LIFT_MS);    // lift, neighbors push down
  moveFeet(pSwing, SWING_MS);   // swing forward
  fx[L] = newFx;
  moveFeet(pLower, LOWER_MS);   // put it down
  return true;
}

// Stagger the feet so each one is at a different point in its step
bool startGait() {
  Serial.println(F("Staggering feet..."));
  for (int k = 0; k < 4; k++) {
    if (!stepLeg(ORDER[k], -STRIDE / 2 + k * STRIDE / 4)) return false;
  }
  phase = 0;
  inGait = true;
  return true;
}

// Bring every foot back to its standing spot, one at a time
bool endGait() {
  Serial.println(F("Returning feet to stand..."));
  for (int k = 0; k < 4; k++) {
    if (!stepLeg(ORDER[k], 0)) return false;
  }
  Vec3 off[4];
  for (int i = 0; i < 4; i++) off[i] = {0, 0, 0};
  moveFeet(off, SHIFT_MS);
  inGait = false;
  return true;
}

void walkStep() {
  int L = ORDER[phase];
  if (!stepLeg(L, fx[L] + STRIDE)) { walking = false; return; }
  for (int i = 0; i < 4; i++) fx[i] -= STRIDE / 4;   // body moves forward a quarter stride
  phase = (phase + 1) % 4;
}

// ---------- commands ----------
void standUp() {
  if (inGait) { Serial.println(F("Send s to stop walking first.")); return; }
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
  for (int i = 0; i < 4; i++) { offsetNow[i] = {0, 0, 0}; fx[i] = 0; }
  Serial.println(F("Standing."));
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "g") { standUp(); return; }
  if (line == "p") { printState(); return; }

  if (line == "w") {
    if (!standing) { Serial.println(F("Send g first to stand.")); return; }
    if (walking) return;
    if (!inGait && !startGait()) return;
    walking = true;
    stopRequested = false;
    Serial.println(F("Walking. Send s to stop."));
    return;
  }

  if (line == "s") {
    if (walking) {
      stopRequested = true;
      Serial.println(F("Stopping at the end of this cycle..."));
    } else if (inGait) {
      if (endGait()) Serial.println(F("Stopped. Standing."));
    }
    return;
  }

  Serial.println(F("Commands: g = stand | w = walk | s = stop | p = print"));
}

void checkSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
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
  Serial.println(F("Commands: g = stand | w = walk | s = stop | p = print"));
}

void loop() {
  checkSerial();
  if (walking) {
    walkStep();
    checkSerial();
    if (walking && phase == 0 && stopRequested) {
      walking = false;
      stopRequested = false;
      if (endGait()) Serial.println(F("Stopped. Standing."));
    }
  }
}