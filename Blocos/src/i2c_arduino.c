#include "i2c_arduino.h"

int fd = 0;                                         //file descriptor of the arduino
float gyro[3], accel[3], mag[3]; 
float current[3]; 
static volatile int counter[3]={0,0,0};             //Hall sensors transition counters

void* read_data(void* arg)
{
    char buf[100];
    int nbytes;
    struct timespec start, stop;
    double accum;
    
    while(1)
    {
    	nbytes = read(fd, buf, 120);
        if(nbytes != 120)                           //error: reattempt reading
            continue;
        sscanf(buf, "%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f", &current[0], &current[1], &current[2], 
                &gyro[0], &gyro[1], &gyro[2], &accel[0], &accel[1], &accel[2], &mag[0], &mag[1], &mag[2]);
    	
    	clock_gettime(CLOCK_REALTIME, &stop);
        accum = ( stop.tv_sec - start.tv_sec ) + ( stop.tv_nsec - start.tv_nsec ) / (float)1000000000;
        if(accum < TS)
            usleep((int)((TS-accum)*1000000));       //sleep remaining time to TS
    	clock_gettime(CLOCK_REALTIME, &start);
    }
}

double get_current(int i)
{
	return current[i]*3.3/2048;
}

double get_gyro(int i)
{
	return gyro[i];
}

double get_accel(int i)
{
	return accel[i];
}

double get_mag(int i)
{
	return mag[i];
}

double get_n(int i)
{
    return counter[i];
}

void Interrupt0 (void) { ++counter[0] ; }           //interrupt routines
void Interrupt1 (void) { ++counter[1] ; }
void Interrupt2 (void) { ++counter[2] ; }

int setup()
{

    pthread_t thread_id;
    if (fd < 1)
    {
        wiringPiSetupGpio();                        //setup I2C communication
        fd = wiringPiI2CSetup(DEVICE_ID);

        pthread_create(&thread_id, NULL, read_data, NULL);

        pinMode(27, OUTPUT);
        pinMode(22, OUTPUT);
        pinMode(9, OUTPUT);
        pinMode(10, OUTPUT);
        pinMode(5, OUTPUT);
        pinMode(13, OUTPUT);

        digitalWrite(27, HIGH);
        digitalWrite(22, LOW);
        digitalWrite(9, HIGH);
        digitalWrite(10, LOW);
        digitalWrite(5, HIGH);
        digitalWrite(13, LOW);

        wiringPiISR (17, INT_EDGE_BOTH, &Interrupt0); //setup Hall sensor ISR routines
        wiringPiISR (11, INT_EDGE_BOTH, &Interrupt1);
        wiringPiISR (6, INT_EDGE_BOTH, &Interrupt2);        
    }
    return fd;
}

void write_pwm(int pwm1, int pwm2, int pwm3, int pwm4)
{
    char buf[25];
    int nbytes;
    int counter = 0;
    
    sprintf(buf, "%d\n%d\n%d\n%d\n", pwm1, pwm2, pwm3, pwm4);
	
    while(counter < 10)                             //attempt writing at most 10 times
    {
        counter ++;
        nbytes = write(fd, buf, 25);
        if (nbytes != 25)                           //error: reattempt reading
            continue;
    	else
    	    break;
    }
}
