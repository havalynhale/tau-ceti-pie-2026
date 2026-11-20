// inverse_v2_balance, micro-ROS version + IMU.
//
// WIRING (ESP32):
//   PCA9685 servo driver:  SDA = GPIO13, SCL = GPIO14   (its own I2C bus, "Wire1")
//   IMU (MPU-6050/6500):   SDA = GPIO21, SCL = GPIO22   (the default I2C bus, "Wire")
//   IMU mounting: chip facing UP, its X arrow pointing to the robot's LEFT,
//                 its Y arrow pointing BACKWARD.  (Change imuToRobot() if you remount it.)
//
// Same IK and balance gait as inverse_v2_balance.ino. Differences:
//   - Commands come in on the ROS topic /cmd (std_msgs/String, one letter)
//     instead of the Serial Monitor. USB serial is now the micro-ROS link.
//   - Every servo move is published on /joint_states (sensor_msgs/JointState)
//     as real joint angles (radians), so RViz can draw the robot live.
//   - Text that used to go to Serial now goes to /status (std_msgs/String).
//   - The board reconnects on its own if the agent is started late or restarted.
//   - NEW: the IMU measures body tilt. It is published on /imu, it stops the walk
//     if the robot tips too far, and it can keep the body level ("leveling").
//
// Commands:  g stand | w walk | s stop | p print | l leveling on/off | c calibrate level
//            z assembly pose (every leg straight out and flat, for attaching servo horns)
//            then: 1-4 pick leg, h/t/k pick hip/thigh/knee, + / - nudge 1 deg, d print offsets
//            x EMERGENCY RELAX: all servos limp instantly, even mid-move (g or z to resume)
//            w r a d , . walk directions | m / n balance margin +/- | ] / [ speed +/-
//
// Agent (WSL): ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
//
// ---------------------------------------------------------------------------
// HOW TO READ THIS FILE (ROS words in plain English)
//
//   topic       = a named channel. Anyone can send ("publish") on it and anyone
//                 can listen ("subscribe"). Like a group chat with a name.
//   message     = one thing sent on a topic. Each topic has a fixed message type,
//                 e.g. std_msgs/String is just text, sensor_msgs/JointState is
//                 a list of joint names + angles.
//   node        = one program on the ROS network. This ESP32 is the node "tau_ceti".
//   agent       = a program running in WSL that relays between this ESP32 (over
//                 USB) and the rest of ROS. The ESP32 can't join ROS by itself;
//                 it is too small, so the agent does the heavy lifting.
//   executor    = the part that checks "did a message arrive?" and, if so, runs
//                 the function we registered for it (a "callback").
//
// The ESP32 talks on four topics:
//   /cmd           IN   text, one letter (from teleop.py)
//   /status        OUT  text, what used to be Serial.println  (shown in teleop.py)
//   /joint_states  OUT  12 joint angles in radians  (RViz draws the legs from these)
//   /imu           OUT  body tilt + raw accel/gyro  (RViz tilts the body from this)
//
// Sections below, top to bottom:
//   1. Constants (original ones + new IMU ones)
//   2. ROS setup + helpers           <- NEW
//   3. IMU: reading tilt, safety stop, leveling   <- NEW
//   4. IK, servo output, gait        <- same as original, with a few hooks into ROS/IMU
//   5. Commands                      <- same logic, but text goes to /status
//   6. Connection to the agent       <- NEW
//   7. setup() / loop()
// ---------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

// =====================================================================
// 1. CONSTANTS
// =====================================================================

// ---------- pins ----------
const int PCA_SDA = 13, PCA_SCL = 14;   // servo driver, on Wire1
const int IMU_SDA = 21, IMU_SCL = 22;   // IMU, on Wire
#define LED_PIN 2                       // on = connected to the agent

// The PCA9685 lives on the SECOND I2C bus (Wire1), not the default one.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

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
// 170 (was 180): 1 cm lower gives the legs ~10 mm more spare reach, which
// sideways walking, turning and quick stick direction changes need.
// (Simulated: at 180 some direction changes over-reach; at 170 none do.)
const float STAND_H = 170.0;
const float STAND_R = 40.0;
const float STAND_SWING[4] = {-30, 30, -30, 30};

// ---------- calibration ----------
const float HIP_SIGN[4] = {1, 1, 1, 1};
const float JOINT_SIGN  = -1;
const float FOOT_SIGN   = -1;
// Not "const": calibration mode (z, then 1-4 / h t k / + -) changes them live.
// Press d to print the final values, then paste them here.
// Calibrated 2026-11-18 with z + nudges after re-attaching the servo horns.
float HIP_OFF[4]   = {73, 80, 90, 73};
// These offsets are the servo angles at which the leg is straight and flat
// (the 'z' assembly pose). Horns re-attached in that pose (2026-11-18), so all legs
// start from the same offsets. b2's joint used to be 70; fine-tune per leg
// here if a joint is a bit off after re-assembly (spline teeth are ~14 deg apart).
float JOINT_OFF[4] = {42, 55, 56, 45};
float FOOT_OFF[4]  = {135, 132, 124, 122};

// ---------- safety limits ----------
const float MAX_YAW      = 60.0;
const float MIN_HIP_DIST = 5.0;
const unsigned long MOVE_MS = 800;

// ---------- walking settings ----------
const float STRIDE      = 30.0;                  // mm the body moves forward per full cycle
const float LIFT        = 35.0;                  // mm the stepping foot lifts
const float SAG_COMP    = 10.0;                  // mm the two neighbor legs push down during a step
// The hand-measured balance shifts below are no longer used directly. They fit
// a balance model EXACTLY (all 4 within 0.1 mm): center of mass 8.5 mm left,
// 1.9 mm back of center, kept 27.5 mm inside the support triangle. The walk
// now uses that model (see shiftFor()), so it keeps the same safety, in every
// walking direction, while only shifting as far as actually needed.
const float SHIFT_LEG[4] = {30, 20, 25, 15};     // measured balance shifts (b1, b2, b3, b4), kept for reference
const float SHIFT_EXTRA = 5.0;                   // extra safety margin on every shift, kept for reference
const float COM_X = -1.9;                        // center of mass, mm forward of body center (from the fit)
const float COM_Y = 8.5;                         // center of mass, mm left of body center (from the fit)
float stabilityMargin = 27.5;   // mm the CoM stays inside the support triangle. Live: m (+) / n (-), D-pad up/down.
                                // Smaller = less rocking but closer to tipping. 27.5 = what you tuned by hand.
float speedK = 1.0;             // walking speed multiplier. Live: ] faster / [ slower, RB / LB. 1.0 = original timing.
const int   ORDER[4]    = {2, 0, 3, 1};          // step order: b3, b1, b4, b2
const unsigned long SHIFT_MS = 1000;             // body slide
const unsigned long LIFT_MS  = 1000;             // foot up
const unsigned long SWING_MS = 800;              // foot forward
const unsigned long LOWER_MS = 1000;             // foot down
const float TURN_PER_CYCLE = 10.0;               // NEW: degrees the body turns per full cycle at full stick
// Stick handling (the Windows script already removes the stick's own wobble):
const float CMD_START      = 0.25;               // push past this to START walking (0..1)
const float CMD_KEEP       = 0.10;               // keep walking until it drops below this (no flicker in between)
const unsigned long STOP_DEBOUNCE_MS = 300;      // stick must stay released this long before it stops
const bool QUICK_STOP = false;   // true = cut a step short when the stick is released (felt finicky;
                                 // off for now: steps always finish, then it pauses)
const unsigned long CMD_VEL_TIMEOUT_MS = 1200;   // no /cmd_vel for this long = controller gone, stop.
                                                 // (The PC bridge already sends "stop" instantly if the
                                                 //  controller disconnects; this only covers the PC side dying.)

// ---------- IMU settings ----------
const uint8_t IMU_ADDR = 0x68;
const float TILT_LIMIT_DEG     = 20.0;  // safety stop: walking halts if the body tips past this
const float COMP_ALPHA         = 0.98;  // tilt filter: 98% gyro (smooth), 2% accel (no drift)
const float LEVEL_KI           = 1.5;   // leveling speed: how fast it corrects (1/s). Too high = wobble
const float LEVEL_DEADBAND_DEG = 1.0;   // ignore tilt smaller than this (stops constant twitching)
const float LEVEL_MAX_DEG      = 12.0;  // never correct more than this, in case something's wrong
const unsigned long LEVEL_IDLE_MS = 40; // while standing still, re-apply leveling every 40 ms

// ---------- ROS settings ----------
// How often we send data to the PC.
// One /joint_states or /imu message is ~300 bytes. USB serial at 115200 baud moves
// ~11,500 bytes/s, so 10 per second of each (~6,000 bytes/s) leaves room for the rest.
const unsigned long JOINT_PUB_MS      = 100;     // 10 Hz while moving
const unsigned long JOINT_IDLE_PUB_MS = 500;     // 2 Hz while still (RViz just needs a heartbeat)
const unsigned long IMU_PUB_MS        = 100;     // 10 Hz always

// ---------- types ----------
struct Vec3 { float x, y, z; };
struct A { float thetaK, thetaT; };
struct Servos { float hip, joint, foot; bool ok; const char* why; };

// ---------- state ----------
bool standing = false;
bool poseKnown = false;       // servos have been written at least once
Vec3 offsetNow[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
Servos cur[4];
float shown[4][3];            // servo angles last written (including mid-ramp), for /joint_states

float fx[4] = {0, 0, 0, 0};   // forward offset of each foot from its standing spot, mm
float fy[4] = {0, 0, 0, 0};   // NEW: sideways offset (left +), mm, for strafing and turning
int  phase = 0;               // which entry of ORDER steps next
bool inGait = false;          // feet are staggered for walking
bool walking = false;
bool stopRequested = false;

// =====================================================================
// 2. ROS SETUP + HELPERS
// =====================================================================

// ---------- micro-ROS objects ----------
// These are the "handles" micro-ROS needs. You make them once in createEntities()
// and throw them away in destroyEntities() if the PC disconnects.
rclc_support_t support;       // the connection/session to the agent
rcl_allocator_t allocator;    // tells micro-ROS how to get memory
rcl_node_t node;              // this robot's identity on the ROS network ("tau_ceti")
rclc_executor_t executor;     // checks for incoming /cmd messages and calls cmdCallback
rcl_publisher_t jointPub;     // sends on /joint_states
rcl_publisher_t statusPub;    // sends on /status
rcl_publisher_t imuPub;       // sends on /imu
rcl_subscription_t cmdSub;    // listens on /cmd
rcl_subscription_t velSub;    // NEW: listens on /cmd_vel (game controller: which way to walk)
geometry_msgs__msg__Twist velMsg;

// Walking direction, -1..1 each: vx forward, vy left, wz turn left.
// Two sources: the game controller (/cmd_vel, must keep arriving or it times
// out to "stop") and the keyboard keys (stay on until s). A moving stick wins.
float joyVx = 0, joyVy = 0, joyWz = 0;
unsigned long lastVelMs = 0;
float kbVx = 0, kbVy = 0, kbWz = 0;
bool kbActive = false;

// Message objects. micro-ROS never allocates memory on its own, so every
// message needs a buffer WE provide (the char arrays below).
std_msgs__msg__String cmdMsg;       // incoming command lands here
char cmdBuf[32];
std_msgs__msg__String statusMsg;    // outgoing status text
char statusBuf[160];

sensor_msgs__msg__JointState jointMsg;   // outgoing joint angles
double jointPos[12];                     // the 12 angles (radians): b1 hip,thigh,knee, b2 ..., b4
rosidl_runtime_c__String jointNames[12]; // the 12 names ("b1_hip_yaw", ...); must match the 3D model
char jointNameBuf[12][16];

sensor_msgs__msg__Imu imuMsg;            // outgoing tilt + accel + gyro

// Are we talking to the agent right now?
enum AgentState { WAITING, CONNECTED };
AgentState agentState = WAITING;

// The /cmd callback only writes the letter here. loop() picks it up and does
// the actual work. (Doing a 10-second walk INSIDE a callback would freeze ROS.)
char pendingCmd = 0;
unsigned long lastJointPub = 0;   // millis() of the last /joint_states send
unsigned long lastImuPub = 0;     // millis() of the last /imu send
unsigned long lastPing = 0;       // millis() of the last "are you there?" to the agent

// ---------- ROS helpers ----------
// Replacement for Serial.println(): sends one line of text on /status.
// teleop.py prints these as "[robot] ...".
void status(const char* text) {
  if (agentState != CONNECTED) return;
  strncpy(statusBuf, text, sizeof(statusBuf) - 1);
  statusBuf[sizeof(statusBuf) - 1] = 0;
  statusMsg.data.data = statusBuf;
  statusMsg.data.size = strlen(statusBuf);
  statusMsg.data.capacity = sizeof(statusBuf);
  rcl_publish(&statusPub, &statusMsg, NULL);
}

// Fill in a message header's timestamp with "now". The ESP32 synced its clock
// with the PC in createEntities(), so this matches PC time and RViz accepts it.
void stampNow(builtin_interfaces__msg__Time &t) {
  int64_t ns = rmw_uros_epoch_nanos();
  t.sec = (int32_t)(ns / 1000000000LL);
  t.nanosec = (uint32_t)(ns % 1000000000LL);
}

// Send the current pose on /joint_states so RViz can draw it.
//
// The servos think in "servo degrees" (90 = middle, with your calibration
// offsets and flipped signs). The 3D model thinks in "real joint angles":
//   hip_yaw = how far the leg is turned left/right of straight-out
//   thigh   = thigh angle above horizontal         (thetaT in the IK)
//   knee    = how far the shin bends from straight (thetaK in the IK)
// legIK() goes  real -> servo:  servo = OFFSET + SIGN * real
// Here we go    servo -> real:  real  = (servo - OFFSET) / SIGN
// We use the servo values actually written, so RViz shows what the servos
// were TOLD to do (including clamping at 13/153), not what we hoped for.
void publishJoints() {
  if (agentState != CONNECTED || !poseKnown) return;   // before the first "g" we don't know the pose
  for (int i = 0; i < 4; i++) {
    float yaw    = (shown[i][0] - HIP_OFF[i])   / HIP_SIGN[i];
    float thetaT = (shown[i][1] - JOINT_OFF[i]) / JOINT_SIGN;
    float thetaK = (shown[i][2] - FOOT_OFF[i])  / FOOT_SIGN;
    jointPos[i * 3 + 0] = radians(yaw);
    jointPos[i * 3 + 1] = radians(thetaT);
    jointPos[i * 3 + 2] = radians(thetaK);
  }
  stampNow(jointMsg.header.stamp);
  rcl_publish(&jointPub, &jointMsg, NULL);
  lastJointPub = millis();
}

void publishImu();   // defined in section 3

// Runs automatically (via the executor) whenever a message arrives on /cmd.
// Keep it tiny: just remember the first letter. loop() does the real work.
// Exception: 'x' (emergency relax) is flagged separately so it's acted on
// right away, even in the middle of a move, instead of waiting its turn.
// NEW: /cmd_vel callback. geometry_msgs/Twist is ROS's standard "move like
// this" message: linear.x forward, linear.y left, angular.z turn left.
// We treat them as -1..1 stick values. Just store them; loop() walks.
void velCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgin;
  joyVx = constrain(m->linear.x,  -1.0, 1.0);
  joyVy = constrain(m->linear.y,  -1.0, 1.0);
  joyWz = constrain(m->angular.z, -1.0, 1.0);
  lastVelMs = millis();
}

extern bool relaxRequested;
void cmdCallback(const void* msgin) {
  const std_msgs__msg__String* m = (const std_msgs__msg__String*)msgin;
  if (m->data.size == 0) return;
  if (m->data.data[0] == 'x') relaxRequested = true;
  else pendingCmd = m->data.data[0];
}

// The original sketch blocks for seconds at a time inside rampTo() (the
// while/delay(20) loop). During that time nothing else runs. rosTick() is
// called from inside those loops so that:
//   - RViz keeps getting joint angles + tilt while the legs move (smooth animation)
//   - an "s" sent mid-step is still received (it gets acted on after the step)
void rosTick() {
  if (agentState != CONNECTED) return;
  if (millis() - lastJointPub >= JOINT_PUB_MS) publishJoints();
  if (millis() - lastImuPub >= IMU_PUB_MS) publishImu();
  rclc_executor_spin_some(&executor, 0);
}

// Fill in the 12 joint names once at boot and point the message at our arrays.
// Names come out as b1_hip_yaw, b1_thigh, b1_knee, b2_hip_yaw, ... b4_knee.
// They MUST match the joint names in ros2/tau_ceti/launch/display.launch.py,
// or RViz won't know which joint each angle belongs to.
void initJointMsg() {
  const char* part[3] = {"hip_yaw", "thigh", "knee"};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      int k = i * 3 + j;
      snprintf(jointNameBuf[k], sizeof(jointNameBuf[k]), "b%d_%s", i + 1, part[j]);
      jointNames[k].data = jointNameBuf[k];
      jointNames[k].size = strlen(jointNameBuf[k]);
      jointNames[k].capacity = sizeof(jointNameBuf[k]);
    }
  }
  memset(&jointMsg, 0, sizeof(jointMsg));
  jointMsg.header.frame_id.data = (char*)"";
  jointMsg.header.frame_id.size = 0;
  jointMsg.header.frame_id.capacity = 1;
  jointMsg.name.data = jointNames;
  jointMsg.name.size = 12;
  jointMsg.name.capacity = 12;
  jointMsg.position.data = jointPos;
  jointMsg.position.size = 12;
  jointMsg.position.capacity = 12;
}

// The IMU message says its data is in the robot body's frame ("base_link"),
// because imuUpdate() already turns the chip's axes into robot axes.
void initImuMsg() {
  memset(&imuMsg, 0, sizeof(imuMsg));
  imuMsg.header.frame_id.data = (char*)"base_link";
  imuMsg.header.frame_id.size = strlen("base_link");
  imuMsg.header.frame_id.capacity = imuMsg.header.frame_id.size + 1;
}

// Join the ROS network: make the node, the 3 publishers, the 1 subscriber,
// and the executor. Called each time the agent is found.
// Returns false if any step fails (we'll just try again in half a second).
//   best_effort publishers (joint_states, imu): fire-and-forget, no waiting for
//       "got it". Fine for data that's resent 10x/second anyway, and never blocks.
//   default (reliable) publisher (status): resends until the PC confirms,
//       so text lines don't go missing.
bool createEntities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "tau_ceti", "", &support) != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(&jointPub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states") != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(&imuPub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&statusPub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&cmdSub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "cmd") != RCL_RET_OK) return false;

  cmdMsg.data.data = cmdBuf;
  cmdMsg.data.size = 0;
  cmdMsg.data.capacity = sizeof(cmdBuf);

  // /cmd_vel is BEST EFFORT: only the newest stick value matters. With
  // "reliable", one lost message made every later one wait for its resend,
  // so the robot saw no stick for over a second at a time (seen in a recording).
  if (rclc_subscription_init_best_effort(&velSub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  // 2 = number of things the executor watches: /cmd and /cmd_vel
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &cmdSub, &cmdMsg, &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &velSub, &velMsg, &velCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

  // /status is "reliable": after each message the ESP32 waits for the PC to
  // confirm it. By default that wait can last up to ~1 s, and during it the
  // ESP32 does nothing else (no stick updates!). That made walking pause at
  // the start of a step. 20 ms is plenty for a USB link.
  rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&statusPub), 20);

  rmw_uros_sync_session(1000);   // so timestamps match ROS time
  return true;
}

// Throw away everything from createEntities() after the agent disappears,
// so we can start clean when it comes back. The "timeout 0" line means
// "don't wait for the (dead) agent to confirm the cleanup".
void destroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&cmdSub, &node);
  rcl_subscription_fini(&velSub, &node);
  rcl_publisher_fini(&jointPub, &node);
  rcl_publisher_fini(&imuPub, &node);
  rcl_publisher_fini(&statusPub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// =====================================================================
// 3. IMU: READING TILT, SAFETY STOP, LEVELING
//
// Angles follow the ROS convention (robot x = forward, y = left, z = up):
//   roll  > 0  = LEFT side is higher than the right
//   pitch > 0  = NOSE is lower than the tail
// =====================================================================

bool imuOk = false;             // false if the IMU didn't answer at boot (walking still works)
int  imuWho = -1;               // chip ID: 0x68 = real MPU-6050, 0x70 = MPU-6500 (yours)
float gyroBias[3] = {0, 0, 0};  // gyro reading when perfectly still (subtracted out), raw units
float accR[3] = {0, 0, 1};      // latest accel in ROBOT axes, in g
float gyrR[3] = {0, 0, 0};      // latest gyro  in ROBOT axes, in rad/s
float roll = 0, pitch = 0;      // filtered tilt vs gravity (rad)
float refRoll = 0, refPitch = 0;// tilt that counts as "level" (set at boot, or with 'c')
unsigned long lastImuUs = 0;

bool levelOn = false;           // leveling switched on with 'l'
float corrRoll = 0, corrPitch = 0;  // how much tilt the legs are currently correcting for (rad)
unsigned long lastLevelApply = 0;

bool tiltGuard = false;         // safety stop armed (only during walking steps)
bool tripped = false;           // safety stop has fired; cleared by 's' or 'g'

// Emergency relax ('x'). The callback sets relaxRequested the instant 'x'
// arrives (even mid-move); doRelax() then cuts every servo signal.
// While relaxed is true, writeLeg() refuses to send anything, so no code path
// can wake the servos up again except an explicit 'g' or 'z'.
bool relaxRequested = false;
bool relaxed = false;
bool inAssembly = false;        // in the 'z' straight-leg pose (calibration keys work); until g or x
void doRelax();                 // defined in section 5

// --- low-level register access (we talk to the chip directly, because the
//     Adafruit MPU6050 library refuses MPU-6500 chips like yours) ---
bool imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int imuReadReg(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// Read accel x,y,z, temperature, gyro x,y,z in one go (14 bytes from register 0x3B)
bool imuReadRaw(int16_t v[7]) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)14) != 14) return false;
  for (int k = 0; k < 7; k++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[k] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

// Turn the chip's axes into the robot's axes. Yours is mounted chip-up with
//   chip X -> robot LEFT (+y),  chip Y -> robot BACK (-x),  chip Z -> robot UP (+z)
// so: robot x = -chip y,  robot y = chip x,  robot z = chip z.
// IF YOU REMOUNT THE IMU, THIS IS THE ONLY PLACE TO CHANGE.
void imuToRobot(const float s[3], float r[3]) {
  r[0] = -s[1];
  r[1] =  s[0];
  r[2] =  s[2];
}

// Tilt from the accelerometer alone (gravity direction). Accurate on average,
// but jumpy whenever the robot moves, which is why we blend it with the gyro.
void accelTilt(const float a[3], float &r, float &p) {
  r = atan2(a[1], a[2]);
  p = atan2(-a[0], sqrt(a[1] * a[1] + a[2] * a[2]));
}

// Set up the chip and measure the gyro's zero point.
// THE ROBOT MUST BE STILL FOR ~1.5 s AFTER POWER-ON / RESET.
// Whatever tilt it has right now becomes "level" (re-do later with 'c').
bool imuInit() {
  imuWho = imuReadReg(0x75);          // WHO_AM_I
  if (imuWho < 0) return false;
  imuWrite(0x6B, 0x80); delay(100);   // reset the chip
  imuWrite(0x6B, 0x01); delay(50);    // wake up, use the gyro's clock (more stable)
  imuWrite(0x1A, 0x03);               // built-in smoothing filter ~44 Hz
  imuWrite(0x1B, 0x00);               // gyro range +-250 deg/s  (131 units per deg/s)
  imuWrite(0x1C, 0x00);               // accel range +-2 g       (16384 units per g)
  if (imuWho != 0x68) imuWrite(0x1D, 0x03);   // MPU-6500 only: accel smoothing ~41 Hz
  delay(50);

  long gsum[3] = {0, 0, 0};
  float asum[3] = {0, 0, 0};
  int n = 0;
  for (int k = 0; k < 200; k++) {     // ~1 second of samples
    int16_t v[7];
    if (imuReadRaw(v)) {
      for (int j = 0; j < 3; j++) { gsum[j] += v[4 + j]; asum[j] += v[j]; }
      n++;
    }
    delay(5);
  }
  if (n < 100) return false;
  for (int j = 0; j < 3; j++) gyroBias[j] = gsum[j] / (float)n;

  float s[3] = {asum[0] / n / 16384.0f, asum[1] / n / 16384.0f, asum[2] / n / 16384.0f};
  imuToRobot(s, accR);
  accelTilt(accR, roll, pitch);
  refRoll = roll;
  refPitch = pitch;
  lastImuUs = micros();
  return true;
}

// Leveling, the core idea:
// If the body is tilted, push the feet on the LOW side down (leg gets longer)
// and the feet on the HIGH side up (leg gets shorter). How much depends on how
// far each foot is from the center:  height change = distance * tilt angle.
// corrRoll/corrPitch slowly grow until the measured tilt is back to "level"
// (that's an integral controller: it keeps pushing as long as there's error).
void levelUpdate(float dt) {
  if (!levelOn || !standing || tripped) return;
  float er = roll - refRoll;
  float ep = pitch - refPitch;
  float db = radians(LEVEL_DEADBAND_DEG);
  float mx = radians(LEVEL_MAX_DEG);
  if (fabs(er) > db) corrRoll  += LEVEL_KI * er * dt;
  if (fabs(ep) > db) corrPitch += LEVEL_KI * ep * dt;
  corrRoll  = constrain(corrRoll,  -mx, mx);
  corrPitch = constrain(corrPitch, -mx, mx);
}

// Extra foot offset (mm) for leg i from the current leveling correction.
//   roll  > 0 (left high):  left feet (y > 0) move UP   -> left side goes down
//   pitch > 0 (nose low):   front feet (x > 0) move DOWN -> nose comes up
Vec3 standTarget(int i);   // defined in section 4
Vec3 levelOffset(int i) {
  Vec3 p = standTarget(i);
  return { 0, 0, p.y * corrRoll - p.x * corrPitch };
}

// Read the chip and update roll/pitch. Called many times a second (from loop()
// and from inside every servo move).
//
// The "complementary filter": the gyro tells us how fast we're rotating, so
// adding (rate * time) gives a smooth angle, but it slowly drifts. The accel
// knows where "down" is, but jumps around when the robot moves. Taking 98%
// gyro + 2% accel each update gives smooth AND drift-free.
void imuUpdate() {
  if (!imuOk) return;
  int16_t v[7];
  if (!imuReadRaw(v)) return;
  unsigned long now = micros();
  float dt = (now - lastImuUs) * 1e-6f;
  lastImuUs = now;

  float sa[3], sg[3];
  for (int j = 0; j < 3; j++) {
    sa[j] = v[j] / 16384.0f;                              // g
    sg[j] = radians((v[4 + j] - gyroBias[j]) / 131.0f);   // rad/s
  }
  imuToRobot(sa, accR);
  imuToRobot(sg, gyrR);

  float ar, ap;
  accelTilt(accR, ar, ap);
  if (dt > 0.2f) {          // long gap since last read: just trust the accel
    roll = ar;
    pitch = ap;
    return;
  }
  roll  = COMP_ALPHA * (roll  + gyrR[0] * dt) + (1 - COMP_ALPHA) * ar;
  pitch = COMP_ALPHA * (pitch + gyrR[1] * dt) + (1 - COMP_ALPHA) * ap;
  levelUpdate(dt);
}

// Safety stop. Only armed during walking steps (tiltGuard), because standing
// up from the floor can legitimately tilt the body for a moment.
// Returns true once tripped; the servo ramps then stop where they are.
bool tiltTrip() {
  if (!tiltGuard || !imuOk) return false;
  if (tripped) return true;
  float r = degrees(roll - refRoll);
  float p = degrees(pitch - refPitch);
  if (fabs(r) > TILT_LIMIT_DEG || fabs(p) > TILT_LIMIT_DEG) {
    tripped = true;
    walking = false;
    stopRequested = false;
    char buf[120];
    snprintf(buf, sizeof(buf), "TILT STOP: roll %.0f, pitch %.0f deg (limit %.0f). Steady it, then send s.",
             r, p, TILT_LIMIT_DEG);
    status(buf);
  }
  return tripped;
}

// Send /imu. Orientation is the tilt RELATIVE TO "level" (so a slightly
// crooked mount still shows as flat). Yaw (heading) is left at 0: without a
// compass it would just drift. imu_tf.py turns this into the body tilt in RViz.
void publishImu() {
  if (agentState != CONNECTED || !imuOk) return;
  float r = roll - refRoll, p = pitch - refPitch;
  float cr = cos(r / 2), sr = sin(r / 2), cp = cos(p / 2), sp = sin(p / 2);
  imuMsg.orientation.w = cr * cp;     // quaternion for (roll, pitch, yaw = 0)
  imuMsg.orientation.x = sr * cp;
  imuMsg.orientation.y = cr * sp;
  imuMsg.orientation.z = -sr * sp;
  imuMsg.angular_velocity.x = gyrR[0];
  imuMsg.angular_velocity.y = gyrR[1];
  imuMsg.angular_velocity.z = gyrR[2];
  imuMsg.linear_acceleration.x = accR[0] * 9.80665;   // ROS wants m/s^2
  imuMsg.linear_acceleration.y = accR[1] * 9.80665;
  imuMsg.linear_acceleration.z = accR[2] * 9.80665;
  stampNow(imuMsg.header.stamp);
  rcl_publish(&imuPub, &imuMsg, NULL);
  lastImuPub = millis();
}

// Everything that has to keep running during a long servo move.
void tick() {
  imuUpdate();
  rosTick();
  if (relaxRequested) doRelax();   // 'x' arrived during a move: stop everything now
}

// =====================================================================
// 4. IK, SERVO OUTPUT, GAIT  (same math as inverse_v2_balance.ino)
//    Changes: writeLeg() remembers what it wrote, ramps call tick() and
//    can be stopped by the tilt safety, and with leveling ON, moves go
//    through rampFeet() so the correction is applied continuously.
// =====================================================================

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
  if (relaxed) return;   // emergency relax: no servo signals until g or z
  writeAngle(HIP_CH[i], hip);
  writeAngle(JOINT_CH[i], joint);
  writeAngle(FOOT_CH[i], foot);
  // Record what was actually sent (clamped), so RViz shows the real servo command
  shown[i][0] = constrain(hip,   ANGLE_MIN, ANGLE_MAX);
  shown[i][1] = constrain(joint, ANGLE_MIN, ANGLE_MAX);
  shown[i][2] = constrain(foot,  ANGLE_MIN, ANGLE_MAX);
}

// NEW: quick-stop hook. While a walking step is running, stepLeg() points this
// at stopWanted(); the ramps check it every 20 ms and stop early if it says so
// (setting rampInterrupted, so stepLeg knows it wasn't a failure).
bool (*rampAbort)() = nullptr;
bool rampInterrupted = false;

// Original ramp: slides each servo angle in a straight line from where it is
// to the target. Used when leveling is OFF, and always for standing up.
// Returns false if the tilt safety stopped it partway.
bool rampTo(Servos target[4], bool sel[4], unsigned long ms) {
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
    tick();             // NEW: keep the IMU + ROS alive during the move
    if (relaxed) return false;   // NEW: emergency relax -> abandon the move
    bool interrupt = rampAbort && rampAbort();   // NEW: quick stop (see stepLeg)
    if (interrupt || tiltTrip()) {   // NEW: freeze right here
      for (int i = 0; i < 4; i++)
        if (sel[i]) cur[i] = { shown[i][0], shown[i][1], shown[i][2], true, "" };
      rampInterrupted = interrupt;
      publishJoints();
      return false;
    }
    if (f >= 1) break;
    delay(20);
  }
  for (int i = 0; i < 4; i++) if (sel[i]) cur[i] = target[i];
  publishJoints();      // NEW: always send the final pose of every move
  return true;
}

// NEW ramp used when leveling is ON: slides the FEET in a straight line
// (instead of the servo angles) and re-solves the IK every 20 ms with the
// latest leveling correction added. That way the body keeps leveling
// even in the middle of a step. If a corrected pose is out of reach,
// that leg just holds its last good position for that tick.
bool rampFeet(Vec3 to[4], unsigned long ms) {
  Vec3 from[4];
  for (int i = 0; i < 4; i++) from[i] = offsetNow[i];
  unsigned long start = millis();
  while (true) {
    float f = (millis() - start) / (float)ms;
    if (f > 1) f = 1;
    for (int i = 0; i < 4; i++) {
      Vec3 o = { from[i].x + (to[i].x - from[i].x) * f,
                 from[i].y + (to[i].y - from[i].y) * f,
                 from[i].z + (to[i].z - from[i].z) * f };
      offsetNow[i] = o;
      Servos s = legIK(i, addVec(addVec(standTarget(i), o), levelOffset(i)));
      if (s.ok) { writeLeg(i, s.hip, s.joint, s.foot); cur[i] = s; }
    }
    tick();
    if (relaxed) return false;
    if (rampAbort && rampAbort()) { rampInterrupted = true; publishJoints(); return false; }
    if (tiltTrip()) { publishJoints(); return false; }
    if (f >= 1) break;
    delay(20);
  }
  publishJoints();
  return true;
}

// While standing still with leveling ON: re-apply the current correction.
// The correction changes slowly (LEVEL_KI), so small direct writes are smooth.
void applyLevelNow() {
  lastLevelApply = millis();
  for (int i = 0; i < 4; i++) {
    Servos s = legIK(i, addVec(addVec(standTarget(i), offsetNow[i]), levelOffset(i)));
    if (s.ok) { writeLeg(i, s.hip, s.joint, s.foot); cur[i] = s; }
  }
}

// Same as delay(ms), but keeps the IMU + ROS alive while waiting.
// Used in standUp() where the original had delay(800).
void rosDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms && !relaxed) { tick(); delay(10); }
}

// ---------- printing (now to /status) ----------
void statusLeg(int i, Servos s) {
  char buf[120];
  if (s.ok) {
    snprintf(buf, sizeof(buf), "b%d  hip %.1f  joint %.1f  foot %.1f  OK", i + 1, s.hip, s.joint, s.foot);
  } else {
    snprintf(buf, sizeof(buf), "b%d  hip %.1f  joint %.1f  foot %.1f  FAIL: %s", i + 1, s.hip, s.joint, s.foot, s.why);
  }
  status(buf);
}

void statusState() {
  for (int i = 0; i < 4; i++) {
    char buf[160];
    snprintf(buf, sizeof(buf), "b%d  offset (%.1f, %.1f, %.1f)  hip %.1f  joint %.1f  foot %.1f",
             i + 1, offsetNow[i].x, offsetNow[i].y, offsetNow[i].z, cur[i].hip, cur[i].joint, cur[i].foot);
    status(buf);
  }
  char buf[160];
  if (imuOk) {
    snprintf(buf, sizeof(buf), "IMU tilt vs level: roll %.1f  pitch %.1f deg | leveling %s, correcting roll %.1f pitch %.1f",
             degrees(roll - refRoll), degrees(pitch - refPitch), levelOn ? "ON" : "OFF",
             degrees(corrRoll), degrees(corrPitch));
  } else {
    snprintf(buf, sizeof(buf), "IMU: not found (check GPIO21/22 wiring)");
  }
  status(buf);
  snprintf(buf, sizeof(buf), "Walk: balance margin %.1f mm, speed %.2fx", stabilityMargin, speedK);
  status(buf);
}

// ---------- moving all feet ----------
// quiet = true: just check, don't report failures (used when trying shorter steps)
bool poseOk(Vec3 off[4], Servos target[4], bool quiet = false) {
  bool ok = true;
  for (int i = 0; i < 4; i++) {
    target[i] = legIK(i, addVec(standTarget(i), off[i]));
    if (!target[i].ok) { if (!quiet) statusLeg(i, target[i]); ok = false; }
  }
  return ok;
}

// Returns false if a pose is unreachable OR the tilt safety stopped the move.
bool moveFeet(Vec3 off[4], unsigned long ms) {
  Servos target[4];
  if (!poseOk(off, target)) return false;
  if (levelOn) return rampFeet(off, ms);    // NEW: leveling path
  bool sel[4] = {true, true, true, true};
  if (!rampTo(target, sel, ms)) return false;
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
// CHANGED: the walk can now go any direction and turn. Each foot has a forward
// offset fx AND a sideways offset fy (the original only had fx). A walking
// command is (vx, vy, wz), each -1..1:
//   vx = forward(+)/back(-), vy = left(+)/right(-), wz = turn left(+)/right(-)
// From that, cycleMove() works out how far EACH foot should travel per full
// 4-step cycle. When turning, feet far from the center travel further.
// The crawl itself (shift body, lift, swing, lower, one leg at a time, same
// leg order) is exactly the original.

// Foot offsets for the current fx/fy, a body shift, and an optional lifted leg
void buildOffsets(Vec3 off[4], Vec3 bs, int liftLeg, float liftZ) {
  for (int i = 0; i < 4; i++) {
    off[i].x = fx[i] - bs.x;
    off[i].y = fy[i] - bs.y;
    off[i].z = 0;
    if (liftLeg >= 0) {
      if (i == liftLeg) off[i].z = liftZ;
      else if (i != diagonalPartner(liftLeg)) off[i].z = -SAG_COMP;
    }
  }
}

// One step: shift body, lift leg L, swing it to (newFx, newFy), lower it.
// Checks all four poses before moving anything.
// NEW: the tilt safety is armed for the whole step; if it fires, we stop
// right where we are and return false.
// Work out the 4 poses of a step (shift, lift, swing, lower) for leg L landing
// at (newFx, newFy). Returns false if any of them is out of reach.
// How far inside triangle (tx, ty) point (qx, qy) is: the distance to the
// nearest edge, negative if outside. The CoM needs this >= stabilityMargin.
float triMargin(float qx, float qy, const float tx[3], const float ty[3]) {
  float best = 1e9;
  for (int k = 0; k < 3; k++) {
    int a = k, b = (k + 1) % 3, o = (k + 2) % 3;
    float ex = tx[b] - tx[a], ey = ty[b] - ty[a];
    float L = sqrt(ex * ex + ey * ey);
    float nx = -ey / L, ny = ex / L;
    if ((tx[o] - tx[a]) * nx + (ty[o] - ty[a]) * ny < 0) { nx = -nx; ny = -ny; }   // point inward
    best = min(best, (qx - tx[a]) * nx + (qy - ty[a]) * ny);
  }
  return best;
}

// NEW body shift before lifting leg L. Same direction as the original
// (bodyShiftDir), but the distance is the SMALLEST that puts the center of
// mass stabilityMargin inside the triangle of the 3 feet that stay down,
// using where those feet ACTUALLY are right now. The original used fixed
// distances measured with the feet in the standing pose, which was more than
// needed walking forward and less than needed walking sideways or turning.
Vec3 shiftFor(int L) {
  float tx[3], ty[3];
  int j = 0;
  for (int k = 0; k < 4; k++) {
    if (k == L) continue;
    Vec3 p = standTarget(k);
    tx[j] = p.x + fx[k];
    ty[j] = p.y + fy[k];
    j++;
  }
  Vec3 n = bodyShiftDir(L);
  float bestT = 0, bestM = -1e9;
  for (int i = 0; i <= 120; i++) {          // try 0 .. 60 mm in 0.5 mm steps
    float t = i * 0.5f;
    float m = triMargin(COM_X + n.x * t, COM_Y + n.y * t, tx, ty);
    if (m >= stabilityMargin) return { n.x * t, n.y * t, 0 };
    if (m > bestM) { bestM = m; bestT = t; }
  }
  return { n.x * bestT, n.y * bestT, 0 };   // can't reach the margin: use the safest spot
}

bool planStep(int L, float newFx, float newFy, Vec3 pShift[4], Vec3 pLift[4],
              Vec3 pSwing[4], Vec3 pLower[4], bool quiet) {
  Vec3 bs = shiftFor(L);   // CHANGED: shift distance from where the feet actually are

  float oldFx = fx[L], oldFy = fy[L];
  buildOffsets(pShift, bs, -1, 0);
  buildOffsets(pLift,  bs, L, LIFT);
  fx[L] = newFx; fy[L] = newFy;
  buildOffsets(pSwing, bs, L, LIFT);
  buildOffsets(pLower, bs, -1, 0);
  fx[L] = oldFx; fy[L] = oldFy;

  Servos tmp[4];
  return poseOk(pShift, tmp, quiet) && poseOk(pLift, tmp, quiet) &&
         poseOk(pSwing, tmp, quiet) && poseOk(pLower, tmp, quiet);
}

// NEW: would this step fit? (checks only, moves nothing, prints nothing)
bool stepFits(int L, float newFx, float newFy) {
  Vec3 a[4], b[4], c[4], d[4];
  return planStep(L, newFx, newFy, a, b, c, d, true);
}

// The legs only have ~16 mm of spare reach when standing, so a full step in
// some directions (or right after the stick flips direction) can be too far.
// Try the full step, then 3/4, 1/2, 1/4. If none fit, return 0: the foot just
// goes back to its standing spot, which re-centers the legs for later steps.
float fitScale(int L, float targetFx, float targetFy) {
  const float tries[4] = {1.0f, 0.75f, 0.5f, 0.25f};
  for (int k = 0; k < 4; k++)
    if (stepFits(L, targetFx * tries[k], targetFy * tries[k])) return tries[k];
  return 0;
}

// NEW: quick stopping. With interruptible = true (normal walking), letting go
// of the stick / pressing stop doesn't wait for the whole ~4 s step:
//   - during the body shift (all 4 feet down): freeze right there
//   - during the lift: put the foot straight back down where it was
//   - during swing / lower: finish (stopping with a foot in mid-air isn't safe)
// An interrupted step returns false with stepInterrupted = true; the walk just
// pauses and the same leg's step starts over when the stick moves again.
bool stepInterrupted = false;
bool getMoveCmd(float &vx, float &vy, float &wz);   // defined in section 5
bool stopWanted() {
  float a, b, c;
  return !getMoveCmd(a, b, c) || stopRequested || relaxRequested;
}

// Say WHY a step got cut short, so a "finicky" stop can be diagnosed:
// stick really released vs. stick signal not arriving vs. stop button.
extern bool joyEngaged;
void reportInterrupt(int L, const char* during) {
  char buf[120];
  unsigned long age = millis() - lastVelMs;
  const char* why = stopRequested ? "stop pressed"
                  : relaxRequested ? "relax"
                  : (age >= CMD_VEL_TIMEOUT_MS && !kbActive) ? "NO STICK SIGNAL"
                  : "stick released";
  snprintf(buf, sizeof(buf), "Paused b%d during %s: %s (last stick msg %lu ms ago, stick %.2f/%.2f/%.2f)",
           L + 1, during, why, age, joyVx, joyVy, joyWz);
  status(buf);
}

bool stepLeg(int L, float newFx, float newFy, bool interruptible = false) {
  stepInterrupted = false;
  Vec3 pShift[4], pLift[4], pSwing[4], pLower[4];
  if (!planStep(L, newFx, newFy, pShift, pLift, pSwing, pLower, false)) {
    char buf[48];
    snprintf(buf, sizeof(buf), "Step for b%d not reachable. Stopping.", L + 1);
    status(buf);
    return false;
  }

  char buf[16];
  snprintf(buf, sizeof(buf), "Step b%d", L + 1);
  status(buf);
  tiltGuard = true;
  rampAbort = interruptible ? stopWanted : nullptr;
  rampInterrupted = false;

  // All phase times are divided by speedK (1.0 = original speed)
  bool ok = moveFeet(pShift, SHIFT_MS / speedK);    // slide body over the other three feet
  if (!ok && rampInterrupted) {            // stopped mid-shift: all feet down, stable as is
    stepInterrupted = true;
  }
  if (!ok && rampInterrupted) reportInterrupt(L, "body shift");
  if (ok) {
    ok = moveFeet(pLift, LIFT_MS / speedK);         // lift, neighbors push down
    if (!ok && rampInterrupted) {          // stopped mid-lift: put the foot back down
      reportInterrupt(L, "lift");
      rampAbort = nullptr;
      moveFeet(pShift, LOWER_MS / 2 / speedK);
      stepInterrupted = true;
    }
  }
  rampAbort = nullptr;                     // from here on, always finish the step
  if (ok) ok = moveFeet(pSwing, SWING_MS / speedK); // swing forward
  if (ok) {
    fx[L] = newFx; fy[L] = newFy;
    ok = moveFeet(pLower, LOWER_MS / speedK);       // put it down
  }
  tiltGuard = false;
  return ok;
}

// How far each foot travels (mm) over one full 4-step cycle for command
// (vx, vy, wz). Translation is the same for every foot. Turning by angle th
// moves each foot along a circle around the body center: new spot = old spot
// rotated by th. Then everything is scaled down if any foot would need to
// travel more than STRIDE (so turning + strafing at once can't over-reach).
void cycleMove(float vx, float vy, float wz, float cx[4], float cy[4]) {
  float th = radians(TURN_PER_CYCLE) * wz;
  float c = cos(th), s = sin(th);
  float longest = 0;
  for (int i = 0; i < 4; i++) {
    Vec3 p = standTarget(i);
    cx[i] = vx * STRIDE + (c * p.x - s * p.y - p.x);
    cy[i] = vy * STRIDE + (s * p.x + c * p.y - p.y);
    longest = max(longest, (float)sqrt(cx[i] * cx[i] + cy[i] * cy[i]));
  }
  if (longest > STRIDE) {
    for (int i = 0; i < 4; i++) { cx[i] *= STRIDE / longest; cy[i] *= STRIDE / longest; }
  }
}

// CHANGED: no more "stagger" (4 setup steps, ~15 s, before walking started).
// Walking now starts straight from the standing pose; the feet spread into
// their walking positions over the first few real steps. Simulated for all
// directions: every step stays in reach and keeps the balance margin.
void startGait() {
  phase = 0;
  inGait = true;
}

// Bring every foot back to its standing spot, one at a time
bool endGait() {
  status("Returning feet to stand...");
  for (int k = 0; k < 4; k++) {
    if (!stepLeg(ORDER[k], 0, 0)) return false;
  }
  Vec3 off[4];
  for (int i = 0; i < 4; i++) off[i] = {0, 0, 0};
  moveFeet(off, SHIFT_MS);
  inGait = false;
  return true;
}

// One step of the walk in direction (vx, vy, wz).
// Same pattern as the original: the stepping foot lands half a cycle AHEAD of
// its standing spot, then every foot slides back a quarter cycle (= the body
// moved a quarter cycle). The original did fx += STRIDE then fx -= STRIDE/4;
// this lands at +cycle/2 directly, which is the same thing when walking
// straight, and also copes with the direction changing between steps.
// NEW: if the full step doesn't fit, a shorter one is used (fitScale), and the
// body moves correspondingly less. Scale 0 = recenter this foot, body stays put.
void walkStep(float vx, float vy, float wz) {
  float cx[4], cy[4];
  cycleMove(vx, vy, wz, cx, cy);
  int L = ORDER[phase];
  float k = fitScale(L, cx[L] / 2, cy[L] / 2);
  // QUICK_STOP = false: each step always finishes before the robot pauses
  // (simpler and smoother; letting go takes up to one step to take effect).
  if (!stepLeg(L, cx[L] / 2 * k, cy[L] / 2 * k, QUICK_STOP)) {
    if (stepInterrupted) return;   // stick let go / stop: pause, same leg goes next time
    walking = false;               // unreachable, tilt stop, or relax
    return;
  }
  for (int i = 0; i < 4; i++) { fx[i] -= cx[i] / 4 * k; fy[i] -= cy[i] / 4 * k; }
  phase = (phase + 1) % 4;
}

// =====================================================================
// 5. COMMANDS  (same as the original handleLine(), but takes one letter
//    from /cmd instead of a line from the Serial Monitor)
// =====================================================================
void standUp() {
  if (inGait) { status("Send s to stop walking first."); return; }
  Servos target[4];
  bool sel[4] = {true, true, true, true};
  for (int i = 0; i < 4; i++) {
    target[i] = legIK(i, standTarget(i));
    if (!target[i].ok) { status("Stand failed:"); statusLeg(i, target[i]); return; }
  }

  if (standing) {
    if (!rampTo(target, sel, MOVE_MS)) return;   // relaxed mid-move
  } else {
    // Legs not yet started show as the 90/90/90 pose in RViz (they're limp anyway)
    if (!poseKnown)
      for (int i = 0; i < 4; i++) { shown[i][0] = shown[i][1] = shown[i][2] = 90; }
    poseKnown = true;   // from now on publishJoints() has real numbers to send
    // CHANGED from the original: each leg snaps to 90/90/90 only when its turn
    // comes, instead of all 12 servos jumping at the same moment. Fewer servos
    // starting at once = smaller current spike through the PCA board's V+ input.
    for (int i = 0; i < 4; i++) {
      cur[i] = {90, 90, 90, true, ""};
      writeLeg(i, 90, 90, 90);
      bool one[4] = {false, false, false, false};
      one[i] = true;
      if (!rampTo(target, one, 2000)) return;   // relaxed mid-move
      rosDelay(800);
      if (relaxed) return;
    }
    standing = true;
  }
  for (int i = 0; i < 4; i++) { offsetNow[i] = {0, 0, 0}; fx[i] = 0; fy[i] = 0; }
  status("Standing.");
}

// 'x': cut the PWM signal on every PCA channel (the "full off" bit), so all
// servos go limp at once and stop drawing current. Resets the robot's state:
// the next g does the full one-leg-at-a-time stand-up from scratch.
void doRelax() {
  relaxRequested = false;
  for (int ch = 0; ch < 16; ch++) pwm.setPWM(ch, 0, 4096);
  relaxed = true;
  walking = false;
  stopRequested = false;
  inGait = false;
  standing = false;
  tripped = false;
  tiltGuard = false;
  levelOn = false;
  corrRoll = corrPitch = 0;
  phase = 0;
  for (int i = 0; i < 4; i++) { fx[i] = 0; fy[i] = 0; offsetNow[i] = {0, 0, 0}; }
  kbActive = false;
  poseKnown = false;    // limp: we no longer know where the legs are
  inAssembly = false;
  pendingCmd = 0;       // drop anything queued behind the x
  status("RELAXED: all servo signals off, servos limp. Press g to stand up again, or z for the straight-leg pose.");
}

// ---------- calibration (assembly pose) ----------
int  calLeg = 0;           // 0..3 = b1..b4
int  calJoint = 0;         // 0 = hip, 1 = thigh, 2 = knee

// Straight-leg pose: each servo AT its offset (= joint angle 0).
// stagger = true (when z is pressed): one leg at a time, 0.3 s apart, so only
// 3 servos start moving at once. Nudges (+/-) don't stagger: they only move 1 deg.
void writeAssemblyPose(bool stagger) {
  for (int i = 0; i < 4; i++) {
    cur[i] = {HIP_OFF[i], JOINT_OFF[i], FOOT_OFF[i], true, ""};
    writeLeg(i, HIP_OFF[i], JOINT_OFF[i], FOOT_OFF[i]);
    if (stagger) { publishJoints(); rosDelay(300); }
    if (relaxed) return;
  }
  publishJoints();
}

// 'd': print the offsets as code lines to paste into the CONSTANTS section.
void printOffsets() {
  char buf[96];
  status("Calibrated offsets (paste over the ones in the code):");
  snprintf(buf, sizeof(buf), "float HIP_OFF[4]   = {%.0f, %.0f, %.0f, %.0f};",
           HIP_OFF[0], HIP_OFF[1], HIP_OFF[2], HIP_OFF[3]);
  status(buf);
  snprintf(buf, sizeof(buf), "float JOINT_OFF[4] = {%.0f, %.0f, %.0f, %.0f};",
           JOINT_OFF[0], JOINT_OFF[1], JOINT_OFF[2], JOINT_OFF[3]);
  status(buf);
  snprintf(buf, sizeof(buf), "float FOOT_OFF[4]  = {%.0f, %.0f, %.0f, %.0f};",
           FOOT_OFF[0], FOOT_OFF[1], FOOT_OFF[2], FOOT_OFF[3]);
  status(buf);
  status("These are already active until the board resets. Press g to stand with them.");
}

void handleCmd(char c) {
  if (c == 'g') { tripped = false; inAssembly = false; relaxed = false; standUp(); return; }
  if (c == 'p') { statusState(); return; }

  // CHANGED: keyboard walking in any direction. Each key sets a direction that
  // keeps going until s (or another direction key). loop() does the stepping.
  //   w forward | r reverse | a turn left | d turn right | , sideways left | . sideways right
  // ('d' only means "done" while in the z calibration pose.)
  if (c == 'w' || c == 'r' || c == 'a' || (c == 'd' && !inAssembly) || c == ',' || c == '.') {
    if (!standing) { status("Send g first to stand."); return; }
    if (tripped) { status("Tilt stop is active. Send s first to return to stand."); return; }
    kbVx = (c == 'w') ? 1 : (c == 'r') ? -1 : 0;
    kbVy = (c == ',') ? 1 : (c == '.') ? -1 : 0;
    kbWz = (c == 'a') ? 1 : (c == 'd') ? -1 : 0;
    kbActive = true;
    stopRequested = false;
    return;
  }

  if (c == 's') {
    kbActive = false;
    if (walking) {
      stopRequested = true;
      status("Stopping...");
    } else if (inGait) {
      tripped = false;
      if (endGait()) status("Stopped. Standing.");
    }
    return;
  }

  // NEW: leveling on/off. Only while not walking, so the switch is smooth.
  if (c == 'l') {
    if (!imuOk) { status("No IMU found, so no leveling."); return; }
    if (walking || inGait) { status("Stop walking first (s), then toggle leveling."); return; }
    levelOn = !levelOn;
    if (!levelOn) {
      corrRoll = corrPitch = 0;
      if (standing) moveFeet(offsetNow, MOVE_MS);   // ease back to the uncorrected pose
    }
    status(levelOn ? "Leveling ON." : "Leveling OFF.");
    return;
  }

  // NEW: assembly pose = every leg STRAIGHT and FLAT:
  //   hip:   pointing straight out from the body center   (yaw   = 0)
  //   thigh: horizontal                                  (thetaT = 0)
  //   shin:  in line with the thigh, also horizontal     (thetaK = 0)
  // The servo angle for "joint angle 0" is exactly the calibration offset
  // (servo = OFFSET + SIGN * 0), so the servos go to HIP_OFF / JOINT_OFF /
  // FOOT_OFF and hold. Attach the horns so each leg is straight
  // and flat, and the offsets are correct by definition. If a leg can only get
  // close (spline teeth), nudge that leg's offset instead of the horn.
  // RViz shows this pose. Afterwards, g does the normal stand-up.
  if (c == 'z') {
    if (walking || inGait) { status("Stop walking first (s)."); return; }
    levelOn = false;
    corrRoll = corrPitch = 0;
    relaxed = false;          // z wakes the servos back up after an x
    if (!poseKnown)           // legs not reached yet show as the target pose in RViz
      for (int i = 0; i < 4; i++) { shown[i][0] = HIP_OFF[i]; shown[i][1] = JOINT_OFF[i]; shown[i][2] = FOOT_OFF[i]; }
    poseKnown = true;
    standing = false;         // so the next g does the full one-leg-at-a-time stand-up
    inAssembly = true;
    writeAssemblyPose(true);  // one leg at a time
    if (relaxed) return;      // x pressed while it was moving
    status("Assembly pose: every leg straight out and flat. Attach horns so each leg matches.");
    status("Fine-tune: 1-4 pick leg, h/t/k pick hip/thigh/knee, + or - moves it 1 deg, d prints the values.");
    return;
  }

  // NEW: calibration keys (only in the assembly pose, after z).
  // Each nudge changes that joint's OFFSET by 1 degree and moves the servo
  // there, so you watch the leg and stop when it's exactly straight + flat.
  if ((c >= '1' && c <= '4') || c == 'h' || c == 't' || c == 'k' ||
      c == '+' || c == '=' || c == '-' || c == 'd') {
    if (!inAssembly) { status("Press z first (calibration only works in the straight-leg pose)."); return; }
    const char* jname[3] = {"hip", "thigh", "knee"};
    if (c >= '1' && c <= '4') calLeg = c - '1';
    else if (c == 'h') calJoint = 0;
    else if (c == 't') calJoint = 1;
    else if (c == 'k') calJoint = 2;
    else if (c == 'd') { printOffsets(); return; }
    else {
      float* off = (calJoint == 0) ? &HIP_OFF[calLeg] : (calJoint == 1) ? &JOINT_OFF[calLeg] : &FOOT_OFF[calLeg];
      float step = (c == '-') ? -1 : 1;
      *off = constrain(*off + step, ANGLE_MIN, ANGLE_MAX);
      writeAssemblyPose(false);
    }
    float val = (calJoint == 0) ? HIP_OFF[calLeg] : (calJoint == 1) ? JOINT_OFF[calLeg] : FOOT_OFF[calLeg];
    char buf[80];
    snprintf(buf, sizeof(buf), "Selected b%d %s, offset = %.0f", calLeg + 1, jname[calJoint], val);
    status(buf);
    return;
  }

  // NEW: live tuning while walking.
  //   m / n (D-pad up/down): balance margin +/- 2.5 mm. Less = less rocking, closer to tipping.
  //   ] / [ (RB / LB):       speed +/- 0.25x. Faster = more servo current at once.
  // Takes effect from the next step. Resets on board reset (put good values in the code).
  if (c == 'm' || c == 'n' || c == ']' || c == '[') {
    char buf[96];
    if (c == 'm' || c == 'n') {
      stabilityMargin = constrain(stabilityMargin + (c == 'm' ? 2.5f : -2.5f), 10.0f, 40.0f);
      snprintf(buf, sizeof(buf), "Balance margin %.1f mm (hand-tuned was 27.5). Lower = less rocking, less safe.",
               stabilityMargin);
    } else {
      speedK = constrain(speedK + (c == ']' ? 0.25f : -0.25f), 0.5f, 2.0f);
      snprintf(buf, sizeof(buf), "Speed %.2fx (1.00 = original). Faster = more current: check PCA temperature.",
               speedK);
    }
    status(buf);
    return;
  }

  // NEW: "the robot is level RIGHT NOW, remember this as level".
  // Use it on flat ground if the IMU is mounted a bit crooked, or if the
  // robot wasn't flat when it powered on.
  if (c == 'c') {
    if (!imuOk) { status("No IMU found."); return; }
    if (levelOn) { status("Turn leveling off (l) before calibrating."); return; }
    refRoll = roll;
    refPitch = pitch;
    status("Level reference set to the current tilt.");
    return;
  }

  status("Commands: g stand | w fwd r back a/d turn ,/. sideways | s stop | x RELAX | p print | l leveling | c level | z straight legs");
}

// Which way to walk right now? Returns false if "don't move".
// A moving controller stick wins (and cancels a keyboard direction);
// otherwise the last keyboard direction key, until s.
//
// Stick rules (to stop it feeling finicky):
//   - START when the stick goes past CMD_START, but KEEP walking until it drops
//     below CMD_KEEP. Thumb wobble in between doesn't stop/start it.
//   - The stick must stay released for STOP_DEBOUNCE_MS before it counts as
//     "stop", so a brief wobble mid-step doesn't trigger the quick-stop.
//   - The stick only sets the DIRECTION: any push = full-length steps.
//     (Half-pushes used to give tiny steps that looked like it wasn't walking.)
//     Speed is RB/LB instead.
//   - No /cmd_vel for CMD_VEL_TIMEOUT_MS (controller gone) = stop immediately.
bool joyEngaged = false;
unsigned long joyLowSince = 0;
float joyDirX = 0, joyDirY = 0, joyDirW = 0;   // last clear direction, normalized

bool getMoveCmd(float &vx, float &vy, float &wz) {
  bool joyFresh = millis() - lastVelMs < CMD_VEL_TIMEOUT_MS;
  float mag = max(fabs(joyVx), max(fabs(joyVy), fabs(joyWz)));
  if (!joyFresh) {
    joyEngaged = false;
  } else if (mag >= CMD_KEEP) {
    joyLowSince = 0;
    if (mag >= CMD_START || joyEngaged) {
      joyEngaged = true;
      joyDirX = joyVx / mag;  joyDirY = joyVy / mag;  joyDirW = joyWz / mag;   // biggest axis = 1
    }
  } else if (joyEngaged) {
    if (joyLowSince == 0) joyLowSince = millis();
    if (millis() - joyLowSince >= STOP_DEBOUNCE_MS) joyEngaged = false;
  }
  if (joyEngaged) {
    kbActive = false;
    vx = joyDirX; vy = joyDirY; wz = joyDirW;
    return true;
  }
  if (kbActive) { vx = kbVx; vy = kbVy; wz = kbWz; return true; }
  vx = vy = wz = 0;
  return false;
}

// If a letter arrived on /cmd since last time, run it now.
void takePendingCmd() {
  if (pendingCmd) {
    char c = pendingCmd;
    pendingCmd = 0;
    handleCmd(c);
  }
}

// =====================================================================
// 6. CONNECTION TO THE AGENT
//    A tiny state machine so you never have to press reset:
//      WAITING   -> every 0.5 s ask "agent, are you there?" (ping).
//                   If yes, join ROS (createEntities) and go to CONNECTED.
//      CONNECTED -> every 1 s ping again. If the agent stops answering
//                   (closed, crashed, USB pulled), clean up and go back to WAITING.
//    The onboard LED shows which state we're in (on = CONNECTED).
// =====================================================================

// Read one register from the PCA9685 (to check it's alive at startup)
int pcaReadReg(uint8_t reg) {
  Wire1.beginTransmission(0x40);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return -1;
  if (Wire1.requestFrom((uint8_t)0x40, (uint8_t)1) != 1) return -1;
  return Wire1.read();
}

// The first messages after connecting: what hardware we found.
void statusHardware() {
  char buf[160];
  int pre = pcaReadReg(0xFE);   // PWM frequency setting; ~133 for 50 Hz means pwm.begin() worked
  if (pre < 0) snprintf(buf, sizeof(buf), "PCA9685: NOT FOUND on GPIO13/14 (servos won't move)");
  else         snprintf(buf, sizeof(buf), "PCA9685: OK on GPIO13/14 (prescale %d)", pre);
  status(buf);
  if (imuOk) {
    snprintf(buf, sizeof(buf), "IMU: OK on GPIO21/22 (id 0x%02X). Level set at boot: roll %.1f, pitch %.1f deg raw",
             imuWho, degrees(refRoll), degrees(refPitch));
  } else {
    snprintf(buf, sizeof(buf), "IMU: NOT FOUND on GPIO21/22. Walking works, but no tilt stop or leveling.");
  }
  status(buf);
}

void manageAgent() {
  if (agentState == WAITING) {
    if (millis() - lastPing < 500) return;
    lastPing = millis();
    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK && createEntities()) {
      agentState = CONNECTED;
      digitalWrite(LED_PIN, HIGH);
      status("tau_ceti connected. Commands: g stand | w walk | s stop | p print | l leveling | c calibrate level");
      statusHardware();
      for (int i = 0; i < 4; i++) statusLeg(i, legIK(i, standTarget(i)));   // IK check, standing pose
    }
  } else {
    if (millis() - lastPing < 1000) return;
    lastPing = millis();
    if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
      destroyEntities();
      agentState = WAITING;
      digitalWrite(LED_PIN, LOW);
    }
  }
}

// =====================================================================
// 7. SETUP / LOOP
// =====================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Hand the USB serial port to micro-ROS. From here on, DON'T use
  // Serial.print anywhere: it would mix garbage into the ROS data stream.
  set_microros_transports();   // USB serial, 115200
  initJointMsg();
  initImuMsg();

  // Servo driver on its own bus. Start the bus on OUR pins first;
  // pwm.begin() then reuses it instead of picking default pins.
  Wire1.begin(PCA_SDA, PCA_SCL);
  Wire1.setTimeOut(10);        // ms: never hang forever if a wire comes loose
  pwm.begin();
  pwm.setOscillatorFrequency(27488192);
  pwm.setPWMFreq(50);
  delay(10);

  // IMU on the default bus. Keep the robot still for ~1.5 s here.
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);       // fast mode: one IMU read takes ~0.4 ms
  Wire.setTimeOut(10);
  imuOk = imuInit();
}

// Each pass through loop():
//   1. make sure we're connected to the agent (do nothing until we are)
//   2. read the IMU
//   3. "spin" = let the executor check for a new /cmd message (waits up to 10 ms)
//   4. run that command if one came in
//   5. if walking, take one step (this takes a few seconds), then check for "s"
//   6. if standing still with leveling on, apply the latest correction
//   7. send heartbeats (/joint_states, /imu) if it's been a while
bool wasMoving = false;   // for the "Walking." / "Paused" messages

void loop() {
  manageAgent();
  if (agentState != CONNECTED) { imuUpdate(); delay(10); return; }

  imuUpdate();
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  if (relaxRequested) doRelax();
  takePendingCmd();

  // CHANGED walking logic (direction from the controller or keyboard):
  //   - stick moved / direction key pressed while standing -> stagger feet, start walking
  //   - while walking: one step per pass, in the CURRENT direction
  //   - stick back to center -> pause with all 4 feet down (still in gait)
  //   - stick released mid-step -> quick-stop at the next safe point (see stepLeg)
  //   - s -> quick-stop, then bring the feet back to stand
  float vx, vy, wz;
  bool moving = getMoveCmd(vx, vy, wz);
  if (moving && standing && !walking && !tripped && !relaxed && !stopRequested) {
    if (!inGait) startGait();
    walking = true;          // "Walking." is reported just below
  }
  if (walking) {
    // Stop right away (the step in progress was already cut short by the
    // quick-stop). endGait() copes with any phase, so no need to finish the cycle.
    if (stopRequested) {
      walking = false;
      stopRequested = false;
      if (endGait()) status("Stopped. Standing.");
    } else if (moving) {
      if (!wasMoving) status("Walking.");
      walkStep(vx, vy, wz);
    } else if (wasMoving) {
      status("Paused (stick released). Push again to walk, X to stand normally.");
    }
    wasMoving = moving && walking;
    rclc_executor_spin_some(&executor, 0);
    takePendingCmd();
  } else {
    wasMoving = false;
  }

  if (levelOn && standing && !walking && !tripped && millis() - lastLevelApply >= LEVEL_IDLE_MS) {
    applyLevelNow();
  }

  if (millis() - lastJointPub >= JOINT_IDLE_PUB_MS) publishJoints();
  if (millis() - lastImuPub >= IMU_PUB_MS) publishImu();
}
