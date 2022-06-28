#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

void set_pwm_dg(uint16_t serwo = 0, double degree = 0)
{
  pwm.setPWM(serwo, 0, (340 * degree) / 180 + 120);
}

void setup() 
{
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  set_pwm_dg(0, 85); // offset -5
  set_pwm_dg(1, 90);
  set_pwm_dg(2, 90);
  
  set_pwm_dg(4, 80); // offset -10
  set_pwm_dg(5, 90);
  set_pwm_dg(6, 90);
  
  set_pwm_dg(8, 95); // offset 5
  set_pwm_dg(9, 90);
  set_pwm_dg(10, 90);
  
  set_pwm_dg(12, 90);
  set_pwm_dg(13, 90);
  set_pwm_dg(14, 90);

}

void loop() 
{

}
