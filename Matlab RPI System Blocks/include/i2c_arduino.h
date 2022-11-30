#include <wiringPiI2C.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <wiringPi.h>

#define DEVICE_ID 0x0b
#define TS 0.01

void* read_data(void* arg);
double get_current(int i);
double get_gyro(int i);
double get_accel(int i);
double get_mag(int i);
double get_n(int i);
int setup();
void write_pwm(int pwm1, int pwm2, int pwm3, int pwm4);
