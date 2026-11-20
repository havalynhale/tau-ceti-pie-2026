#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define USMIN       500 //500     // servo minimum pulse (µs)
#define USMAX       2500 //2500    // servo maximum pulse (µs)
#define USCENTER    1500    // center pulse (µs)
#define SERVO_FREQ  300 //300     // Hz, within the servo's 50–330 Hz spec
#define OSC_FREQ    26034545UL  // replace with your calibrated value

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // address 0x40

uint8_t servoChannel = 2;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(OSC_FREQ); // must come before setPWMFreq
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  pwm.writeMicroseconds(servoChannel, USCENTER);
  delay(1000);
}

void loop() {
  pwm.writeMicroseconds(2, 1300);
  pwm.writeMicroseconds(3, 1300);
  pwm.writeMicroseconds(4, 1300);
    


}