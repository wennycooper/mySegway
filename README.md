# mySegway #
This is the control code of the segway 

1. access mpu6050 to get gyro and acceleration information
2. compute y-axis error with complimentary filter
3. update dc motor speed with PID control

## Requirements
1. Raspberry Pi
2. wiringPi 


## Build Instruction 
gcc -o mySegway ./mySegway.c ./motors.c  -lwiringPi -lpthread -lm


## Run 
./mySegway

## Reference: ##
http://blog.bitify.co.uk/2013/11/interfacing-raspberry-pi-and-mpu-6050.html  
http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html  
https://projects.drogon.net/raspberry-pi/wiringpi/software-pwm-library/  
http://robotrabbit.blogspot.tw/2012/07/pid.html  
http://www.bajdi.com/building-a-self-balancing-bot/  





