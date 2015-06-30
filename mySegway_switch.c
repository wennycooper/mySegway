#include  <wiringPiI2C.h>
#include  <sys/types.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <sys/time.h>
#include  <string.h>


// PID parameters
double Kp = 4.0;  //8.0
double Ki = 15;   // 1.2/4.0
double Kd = 0.12; //15.0*2.0; //10.0
double K  = 1.0;
//double K  = 1.9*1.12;


// Complimentary Filter parameters
double K0 = (double) 0.98;
double K1 = (double) 0.02;

int fd;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;


double gyro_offset_x, gyro_offset_y;
double gyro_total_x, gyro_total_y;
double gyro_x_delta, gyro_y_delta;
double rotation_x, rotation_y;
double last_x, last_y;

struct timeval tv, tv2;
unsigned long long      timer, t;

double deltaT;

int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr+1);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}

double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}

double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}

void read_all()
{
    acclX = read_word_2c(0x3B);
    acclY = read_word_2c(0x3D);
    acclZ = read_word_2c(0x3F);

    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;

    gyroX = read_word_2c(0x43);
    gyroY = read_word_2c(0x45);
    gyroZ = read_word_2c(0x47);

    gyro_scaled_x = gyroX / 131.0;
    gyro_scaled_y = gyroY / 131.0;
    gyro_scaled_z = gyroZ / 131.0;
}

unsigned long long  getTimestamp()
{
  gettimeofday(&tv, NULL);
  return (unsigned long long) tv.tv_sec * 1000000 + tv.tv_usec;
}


double constrain(double v, double min_v, double max_v)
{
  if (v <= min_v)
    return (double)min_v;
  else if (v >= max_v)
    return (double)max_v;
  else
    return (double)v;
}

double error, last_error, integrated_error;
double pTerm, iTerm, dTerm;
double angle;
double angle_offset = 0.0;

double speed;
double left_offset = 0.0;
double right_offset = 0.0;
double forward_offset = 0.0;

void pid()
{
  error = last_y + angle_offset - forward_offset;

  pTerm = Kp * error;

  integrated_error = 0.99*integrated_error + error * deltaT ;
  iTerm = Ki * integrated_error;
  //iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);

  dTerm = Kd * (error - last_error) / deltaT;
  last_error = error;

  speed = constrain(K*(pTerm + iTerm + dTerm), -100.0, +100.0); 
  
}

int fd_bt;
char buf[1024];
char BT_DEV[] = "/dev/ttyAMA0";

void init_bt()
{
  if ((fd_bt = open(BT_DEV, O_RDWR | O_NONBLOCK)) == -1)
  {
    //printf("NonBlocking device %s open failed!! \n", BT_DEV);
    exit(EXIT_FAILURE);
  }
  else
  {
    //printf("NonBlocking device %s open OK!! \n", BT_DEV);
  }
}

#define CMD_START 1
#define CMD_STOP  2
#define CMD_FORWARD  3
#define CMD_BACKWARD 4

int wait_for_start()
{
  while(1) {
    if (read(fd_bt, buf, 1024) != -1)
    {
      //printf("%s(%d)\n", buf, strlen(buf));
      if (strncmp(buf, "START", 5) == 0) {
        //printf("MATCHED START\n");
        return 1;
      }
    }
  }
}

int check_for_cmd()
{
  if (read(fd_bt, buf, 1024) != -1)
  {
//    printf("%s(%d)\n", buf, strlen(buf));
//    if (strncmp(buf, "START", 5) == 0)
//       printf("MATCHED START\n");
    if (strncmp(buf, "STOP", 4) == 0) {
//       printf("MATCHED STOP\n");
       return CMD_STOP;
    }

    if (strncmp(buf, "LEFT_TOUCHDOWN", 14) == 0) {
       right_offset = 10.0;
       left_offset = -10.0;
    }
    if (strncmp(buf, "LEFT_TOUCHUP", 12) == 0) {
       right_offset = 0.0;
       left_offset = 0.0;
    }

    if (strncmp(buf, "RIGHT_TOUCHDOWN", 15) == 0) {
       right_offset = -10.0;
       left_offset = 10.0;
    }
    if (strncmp(buf, "RIGHT_TOUCHUP", 13) == 0) {
       right_offset = 0.0;
       left_offset = 0.0;
    }

    if (strncmp(buf, "FORWARD_TOUCHDOWN", 17) == 0) {
//       printf("MATCHED FORWARD_TOUCHDOWN\n");
       forward_offset = 5.0;
    }
    if (strncmp(buf, "FORWARD_TOUCHUP", 15) == 0) {
//       printf("MATCHED FORWARD_TOUCHUP\n");
       forward_offset = 0.0;
    }
    if (strncmp(buf, "BACKWARD_TOUCHDOWN", 18) == 0) {
//       printf("MATCHED BACKWARD_TOUCHDOWN\n");
       forward_offset = -5.0;
    }
    if (strncmp(buf, "BACKWARD_TOUCHUP", 16) == 0) {
//       printf("MATCHED BACKWARD_TOUCHUP\n");
       forward_offset = 0.0;
    }


  }
  return 0;
}

int main()
{
  init_bt();

init_point:

  init_motors();
  delay(5);
  integrated_error = 0.0;
  last_error = 0.0;
  

  // wait for START
  wait_for_start();

  fd = wiringPiI2CSetup (0x68);
  wiringPiI2CWriteReg8 (fd,0x6B,0x00);//disable sleep mode 
//  printf("set 0x6B=%X\n",wiringPiI2CReadReg8 (fd,0x6B));

  timer = getTimestamp();

  deltaT = (double) (getTimestamp() - timer)/1000000.0;
  read_all();

  last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
  last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);


  gyro_offset_x = gyro_scaled_x;
  gyro_offset_y = gyro_scaled_y;

  gyro_total_x = last_x - gyro_offset_x;
  gyro_total_y = last_y - gyro_offset_y;


  while(1) {
    t = getTimestamp();
    deltaT = (double) (t - timer)/1000000.0;
    timer = t;

    read_all();

    gyro_scaled_x -= gyro_offset_x;
    gyro_scaled_y -= gyro_offset_y;

    gyro_x_delta = (gyro_scaled_x * deltaT);
    gyro_y_delta = (gyro_scaled_y * deltaT);

    gyro_total_x += gyro_x_delta;
    gyro_total_y += gyro_y_delta;

    rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

//    printf("[BEFORE] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y= %f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);

//    printf("[1st part] = %f\n", (double) K0*(last_y + gyro_y_delta));
//    printf("[2nd part] = %f\n", (double) K1*rotation_y);
    last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x);
    last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y);

//    printf("[AFTER] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y=%f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);
    
    if (last_y < -45.0 || last_y > 45.0) {
      stop_motors();
      break;
    }

    pid();
//    printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", error, speed, pTerm, iTerm, dTerm, deltaT);

    motors(speed, left_offset, right_offset);

    if (check_for_cmd() == CMD_STOP) {
//      printf("WE SHOULD STOP NOW\n");
      stop_motors();
      //break;
      goto init_point;
    }

    //delay(10);
  }

  stop_motors();
  goto init_point;

  //return 0;
}
