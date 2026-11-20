#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Degree scale: 0 = 670 us, 180 = 2470 us (10 us per degree).
// Actual pulses are limited to 800-2200 us (13 to 153) below.
const int US_MIN = 670;
const int US_MAX = 2470;
const float ANGLE_MIN = 13.0;   // 800 us
const float ANGLE_MAX = 153.0;  // 2200 us

// ---------- channels (leg order b1, b2, b3, b4) ----------
const int HIP_CH[4]   = {1, 4, 8, 12};
const int JOINT_CH[4] = {2, 5, 9, 13};
const int FOOT_CH[4]  = {3, 6, 10, 14};

// ---------- geometry (mm, degrees) ----------
const float L1   = 118.25;   // thigh: joint shaft to knee
const float L2   = 81.81;    // shin: knee to foot tip
const float R    = 76.98;    // body center to hip shaft
const float COXA = 55.0;     // hip shaft to joint shaft, horizontal
const float LEG_ANGLE[4] = {72, 288, 144, 216};

// Standing foot target, measured from the joint shaft
const float STAND_H = 185.0; // down
const float STAND_R = 55.0;  // out

// ---------- calibration ----------
const float HIP_SIGN   = 1;
const float JOINT_SIGN = -1;
const float FOOT_SIGN  = -1;
const float HIP_OFF[4]   = {90, 90, 90, 90};
const float JOINT_OFF[4] = {75, 75, 90, 85};   // thigh horizontal
const float FOOT_OFF[4]  = {85, 75, 90, 85};   // leg straight

// ---------- safety limits ----------
const float MAX_OFFSET   = 60.0;   // mm, biggest offset from standing per axis
const float MAX_YAW      = 45.0;   // degrees, biggest hip turn from 90
const float MIN_HIP_DIST = 5.0;    // mm, foot must be this far from hip axis
const unsigned long MOVE_MS = 600; // time for each move

// ---------- types ----------
struct Vec3 { float x, y, z; };
struct A { float thetaK, thetaT; };
struct Servos { float hip, joint, foot; bool ok; const char* why; };

// ---------- state ----------
bool standing = false;
Vec3 offsetNow[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
Servos cur[4];

// ---------- IK functions ----------
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
  s.hip   = HIP_OFF[i]   + HIP_SIGN   * yaw;
  s.joint = JOINT_OFF[i] + JOINT_SIGN * degrees(a.thetaT);
  s.foot  = FOOT_OFF[i]  + FOOT_SIGN  * degrees(a.thetaK);

  if (s.ok && fabs(yaw) > MAX_YAW) { s.ok = false; s.why = "hip turn too big"; }
  if (s.ok && !(inRange(s.hip) && inRange(s.joint) && inRange(s.foot))) {
    s.ok = false; s.why = "servo past 13-153";
  }
  return s;
}

Vec3 standTarget(int i) {
  float V = radians(LEG_ANGLE[i]);
  float out = R + COXA + STAND_R;
  return { out * cos(V), out * sin(V), -STAND_H };
}

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

// Ramps the selected legs from their current angles to the targets together.
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

// ---------- parsing ----------
// Pulls up to maxN numbers out of text like "(10, -5, 20)" or "10 -5 20".
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
    rampTo(target, sel, MOVE_MS);              // already up: glide back to stand
  } else {
    for (int i = 0; i < 4; i++) {              // first time: one leg at a time from 90
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
    Serial.println(F("Use: 1 (x,y,z)  |  a (x,y,z)  |  g  |  p"));
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

  Servos target[4];
  bool allOk = true;
  for (int i = 0; i < 4; i++) {
    target[i] = cur[i];
    if (!sel[i]) continue;
    Vec3 P = standTarget(i);
    P.x += off.x;  P.y += off.y;  P.z += off.z;
    target[i] = legIK(i, P);
    printLeg(i, target[i]);
    if (!target[i].ok) allOk = false;
  }

  if (!allOk) {
    Serial.println(F("Rejected. Not moving."));
    return;
  }

  rampTo(target, sel, MOVE_MS);
  for (int i = 0; i < 4; i++) if (sel[i]) offsetNow[i] = off;
  Serial.println(F("Moved."));
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line == "g") { standUp(); return; }
  if (line == "p") { printState(); return; }
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
  Serial.println(F("Send g to stand, then e.g. 1 (0,0,20)"));
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}