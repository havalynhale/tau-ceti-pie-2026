// Wiring check: tests each I2C wire at the pin level, then scans both I2C buses
// and confirms the PCA9685 and MPU6050 answer.
//   Bus 0 (Wire):  SDA=21, SCL=22  -> MPU6050 expected at 0x68
//   Bus 1 (Wire1): SDA=13, SCL=14  -> PCA9685 expected at 0x40 (0x70 is its "all call" address, normal)
// Open a serial monitor at 115200. Prints a report every 3 seconds.

#include <Wire.h>

const int MPU_SDA = 21, MPU_SCL = 22;
const int PCA_SDA = 13, PCA_SCL = 14;

// An idle I2C line should sit HIGH (the modules have pull-up resistors).
// LOW with nothing driving it = wire to GND, a short, or an unpowered module.
void pinReport(int pin, const char *name) {
  pinMode(pin, INPUT);                 // no ESP32 pull-up: shows the module's own pull-up
  delay(2);
  int floating = digitalRead(pin);
  pinMode(pin, INPUT_PULLUP);          // weak ESP32 pull-up
  delay(2);
  int pulled = digitalRead(pin);
  Serial.printf("  %-8s GPIO%-2d  no-pullup=%s  with-pullup=%s  -> %s\n", name, pin,
                floating ? "HIGH" : "LOW ", pulled ? "HIGH" : "LOW ",
                (floating && pulled) ? "OK (module pull-up present)"
                : pulled ? "no module pull-up (module unpowered or wire not connected?)"
                : "STUCK LOW (shorted to GND, or module powered wrong)");
}

// Drive one pin LOW and see if another follows it = the two wires are touching
void shortCheck(int a, int b, const char *na, const char *nb) {
  pinMode(b, INPUT_PULLUP);
  pinMode(a, OUTPUT); digitalWrite(a, LOW);
  delay(2);
  int follows = digitalRead(b) == LOW;
  pinMode(a, INPUT_PULLUP);
  if (follows) Serial.printf("  SHORT: %s (GPIO%d) and %s (GPIO%d) are connected together!\n", na, a, nb, b);
}

void scan(TwoWire &bus, const char *name) {
  Serial.printf("%s: ", name);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) { Serial.printf("0x%02X ", a); n++; }
  }
  if (n == 0) Serial.print("nothing found");
  Serial.println();
}

int readReg(TwoWire &bus, uint8_t addr, uint8_t reg) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return -1;
  if (bus.requestFrom(addr, (uint8_t)1) != 1) return -1;
  return bus.read();
}

bool mpuBusOk = true, pcaBusOk = true;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("==== PIN CHECK (before I2C starts) ====");
  pinReport(MPU_SDA, "MPU SDA"); pinReport(MPU_SCL, "MPU SCL");
  pinReport(PCA_SDA, "PCA SDA"); pinReport(PCA_SCL, "PCA SCL");
  int pins[4] = {MPU_SDA, MPU_SCL, PCA_SDA, PCA_SCL};
  const char *names[4] = {"MPU SDA", "MPU SCL", "PCA SDA", "PCA SCL"};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      if (i != j) shortCheck(pins[i], pins[j], names[i], names[j]);

  // Only start a bus whose lines idle HIGH, so a stuck bus can't hang the test
  pinMode(MPU_SDA, INPUT); pinMode(MPU_SCL, INPUT);
  pinMode(PCA_SDA, INPUT); pinMode(PCA_SCL, INPUT);
  delay(2);
  mpuBusOk = digitalRead(MPU_SDA) && digitalRead(MPU_SCL);
  pcaBusOk = digitalRead(PCA_SDA) && digitalRead(PCA_SCL);
  if (mpuBusOk) { Wire.begin(MPU_SDA, MPU_SCL); Wire.setTimeOut(20); }
  if (pcaBusOk) { Wire1.begin(PCA_SDA, PCA_SCL); Wire1.setTimeOut(20); }
  Serial.printf("MPU bus %s, PCA bus %s\n\n", mpuBusOk ? "started" : "SKIPPED (lines not HIGH)",
                pcaBusOk ? "started" : "SKIPPED (lines not HIGH)");
}

void loop() {
  Serial.println("==== I2C CHECK ====");
  if (mpuBusOk) scan(Wire,  "bus0 (21/22, MPU)");
  if (pcaBusOk) scan(Wire1, "bus1 (13/14, PCA)");

  // MPU6050: WHO_AM_I register 0x75 should read 0x68 (some clones say 0x70/0x72/0x98)
  int who = mpuBusOk ? readReg(Wire, 0x68, 0x75) : -1;
  if (who < 0) {
    Serial.println("MPU6050: NOT RESPONDING at 0x68");
  } else {
    Serial.printf("MPU6050: WHO_AM_I = 0x%02X\n", who);
    // Wake it up (PWR_MGMT_1 = 0) and read accel; flat & still should be ~1.0 g on one axis
    Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
    delay(50);
    Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)6);
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Serial.printf("MPU6050: accel g = x %.2f  y %.2f  z %.2f\n", ax / 16384.0, ay / 16384.0, az / 16384.0);
  }

  // PCA9685: MODE1 register 0x00 answers if the chip is alive
  int mode1 = pcaBusOk ? readReg(Wire1, 0x40, 0x00) : -1;
  if (mode1 < 0) Serial.println("PCA9685: NOT RESPONDING at 0x40");
  else           Serial.printf("PCA9685: MODE1 = 0x%02X (responding)\n", mode1);

  Serial.println();
  delay(3000);
}
