#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#define RANGE	100
#define INITIAL_VALUE 0

void init_motors()
{
  wiringPiSetup();

  // initialize left motor (pin0, 2 & 3)
  softPwmCreate(0, INITIAL_VALUE, RANGE);
  softPwmCreate(2, INITIAL_VALUE, RANGE);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  // initialize right motor (pin4, 5 & 6)
  softPwmCreate(5, INITIAL_VALUE, RANGE);
  softPwmCreate(4, INITIAL_VALUE, RANGE);
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
}

void stop_motors()
{
  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(0, LOW);
  digitalWrite(2, LOW);

  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(4, LOW);

//  printf("motor stopped!!\n");

  // exit(1);

}

double left_speed;
double right_speed;

void motors(double speed, double left_offset, double right_offset)
{

// to come to me, drive pin 0&5 to some power
// to away from me, drive pin 2&4 to some power

// pin 0:LM+, 2:LM-, 3:LCE
// pin 5:RM+, 4:RM-, 6:RCE
// put M+ high & M- low will come to me
// put M+ low & M- High will away from me

  left_speed = speed + left_offset;
  right_speed = speed + right_offset;

  // left motor
  if (left_speed < 0)  {
    softPwmWrite(0, (int) -left_speed);
    softPwmWrite(2, 0);
  }
  else
  if (left_speed > 0)  {
    softPwmWrite(2, (int) left_speed);
    softPwmWrite(0, 0);
  }

  // right motor
  if (right_speed < 0)  {
    softPwmWrite(5, (int) -right_speed);
    softPwmWrite(4, 0);
  }
  else
  if (right_speed > 0)  {
    softPwmWrite(4, (int) right_speed);
    softPwmWrite(5, 0);
  }
}



