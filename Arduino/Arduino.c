#include <Wire.h>                     //I2C communication
#include <string.h>                   //sscanf and sprintf
#include <stdio.h>              
#include <inttypes.h>                 //sprintf adcBuffer: PRId16 macro
#include <Arduino_LSM9DS1.h>          //IMU functions
#include <nrfx_pwm.h>                 //custom PWM

#define I2C_SLAVE_ADDRESS 11          //Arduino is a slave on address 0x0b
#define SAMPLES_PER_SECOND  100       //100Hz sample of current
#define PPI_CHANNEL         (7)
#define ADC_BUFFER_SIZE     3         //3 current measurements
#define PWM_COUNTER_TOP 1024          //PWM is a value between 0 and 1023
#define RW_ONLY 0                     //[RW_roll, RW_pitch, RW_yaw]
#define LINEAR_ONLY 1                 //[linear_motor_x, linear_motor_y, linear_motor_z]
#define HYBRID  2                     //[linear_motor_x, linear_motor_y, RW_YAW]

//ADC global variable where the values read by the analog pins are stored
volatile nrf_saadc_value_t adcBuffer[ADC_BUFFER_SIZE];

//I2C communication global variables
char buf[25];
char response[120];
int pwm[3];
int mode;
float gyro[3], accel[3], mag[3];

//PWM global variables
static nrfx_pwm_t pwm1 = NRFX_PWM_INSTANCE(0);
static nrfx_pwm_t pwm2 = NRFX_PWM_INSTANCE(1);
static nrf_pwm_values_individual_t seq1_values[] = {0, 0, 0, 0};
static nrf_pwm_values_individual_t seq2_values[] = {0, 0, 0, 0};
int spin[6] = {3,5,7,9,11,13};

static nrf_pwm_sequence_t seq1 = {
    .values = {
        .p_individual = seq1_values
    },
    .length          = NRF_PWM_VALUES_LENGTH(seq1_values),
    .repeats         = 1,
    .end_delay       = 0
};

static nrf_pwm_sequence_t seq2 = {
    .values = {
        .p_individual = seq2_values
    },
    .length          = NRF_PWM_VALUES_LENGTH(seq2_values),
    .repeats         = 1,
    .end_delay       = 0
};

void setup()
{ 
  if (!IMU.begin()) {                       //IMU init 
    while (1);
  }

  initADC();                                //ADC init
  initTimer4();
  initPPI();

  Wire.begin(I2C_SLAVE_ADDRESS);            //I2C init
  Wire.setClock(400000);
  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);

  pwm_setup();                              //PWM init
  for(int i=0; i<6; i++)
    pinMode(spin[i], OUTPUT);    
}

void loop(){
  delay(100);  
}

/*------------------------------------------------------------------------------*/
/*                          I2C functions                                       */
/*------------------------------------------------------------------------------*/

void requestEvents()                  //each time the RPi calls the read function
{
    read_imu(gyro, accel, mag);
    sprintf(response, "%" PRId16 "\n%" PRId16 "\n%" PRId16 "\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n",
            adcBuffer[0], adcBuffer[1], adcBuffer[2], gyro[0], gyro[1], gyro[2],
            accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
    Wire.write(response,120);         //protocol: fixed size 120 bytes message from INO to RPI
}

void receiveEvents(int numBytes)      //each time the RPi calls the write function
{
  for (int i = 0; i < 25; i++)        //protocol: fixed size 25 bytes message from RPI to INO
    buf[i] = Wire.read();
  sscanf(buf, "%d\n%d\n%d\n%d\n", &mode, &pwm[0], &pwm[1], &pwm[2]);
  write_pwm();  
}

/*------------------------------------------------------------------------------*/
/*                          PWM functions                                       */
/*------------------------------------------------------------------------------*/

void pwm_setup()
{
  nrfx_pwm_config_t config1 = {         //PWM channel 1 @ 15.625KHz, [0, 1023]
    .output_pins  = {
        32+ 11,   // Arduino pin 2
        32+ 15,   // Arduino pin 4
        32+ 14,   //Arduino pin 6
        NRFX_PWM_PIN_NOT_USED,
    },
    .irq_priority = 7,
    .base_clock   = NRF_PWM_CLK_16MHz,
    .count_mode   = NRF_PWM_MODE_UP,
    .top_value    = PWM_COUNTER_TOP,
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode    = NRF_PWM_STEP_AUTO,
  };
  nrfx_pwm_init(&pwm1, &config1, NULL);

  nrfx_pwm_config_t config2 = {         //PWM channel 2 @ 15.625KHz, [0, 1023]
    .output_pins  = {
        0 + 21, //Arduino pin 8
        32 + 2, //Arduino pin 10
        32 + 8, //Arduino pin 12
        NRFX_PWM_PIN_NOT_USED,
    },
    .irq_priority = 7,
    .base_clock   = NRF_PWM_CLK_16MHz,
    .count_mode   = NRF_PWM_MODE_UP,
    .top_value    = PWM_COUNTER_TOP,
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode    = NRF_PWM_STEP_AUTO,
  };
  nrfx_pwm_init(&pwm2, &config2, NULL);  
}

int get_signal(int* pwm)                //saturates pwm to [0, 1023]
{
  int s;                                //returns value to be put in signal pin
  if (*pwm < 0)
  {
    s = HIGH;
    *pwm = - *pwm;
  } else
    s = LOW;

  if (*pwm > PWM_COUNTER_TOP -1)
    *pwm = PWM_COUNTER_TOP -1;
  return s;
}

int write_pwm()                         //writes pwm value changing seq*_values and
{                                       //calling nrfx_pwm_simple_playback
  switch(mode)
  {
    case RW_ONLY:
      for(int i=0; i<3; i++)
          digitalWrite(spin[i], get_signal(&pwm[i]));                            
      (*seq1_values).channel_0 = PWM_COUNTER_TOP -1 - pwm[0];
      (*seq1_values).channel_1 = PWM_COUNTER_TOP -1 - pwm[1];
      (*seq1_values).channel_2 = PWM_COUNTER_TOP -1 - pwm[2];
      (void)nrfx_pwm_simple_playback(&pwm1, &seq1, 1, NRFX_PWM_FLAG_LOOP);  
      break;
    case LINEAR_ONLY:
      for(int i=0; i<3; i++)
        digitalWrite(spin[i+3], get_signal(&pwm[i]));   
      (*seq2_values).channel_0 = PWM_COUNTER_TOP -1 - pwm[0];
      (*seq2_values).channel_1 = PWM_COUNTER_TOP -1 - pwm[1];
      (*seq2_values).channel_2 = PWM_COUNTER_TOP -1 - pwm[2];
      (void)nrfx_pwm_simple_playback(&pwm2, &seq2, 1, NRFX_PWM_FLAG_LOOP);  
      break;
    case HYBRID:
      digitalWrite(spin[3], get_signal(&pwm[0]));   
      digitalWrite(spin[4], get_signal(&pwm[1]));   
      digitalWrite(spin[2], get_signal(&pwm[2]));   
      (*seq2_values).channel_0 = PWM_COUNTER_TOP -1 - pwm[0];
      (*seq2_values).channel_1 = PWM_COUNTER_TOP -1 - pwm[1];
      (*seq1_values).channel_2 = PWM_COUNTER_TOP -1 - pwm[2];
      (void)nrfx_pwm_simple_playback(&pwm1, &seq1, 1, NRFX_PWM_FLAG_LOOP);  
      (void)nrfx_pwm_simple_playback(&pwm2, &seq2, 1, NRFX_PWM_FLAG_LOOP);  
      break;
  }  
  return 0;
}

/*------------------------------------------------------------------------------*/
/*                          IMU functions                                       */
/*------------------------------------------------------------------------------*/

void read_imu(float* g, float* a, float* m)
{
  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(g[0], g[1], g[2]);
  if (IMU.accelerationAvailable())
    IMU.readAcceleration(a[0], a[1], a[2]);
  if (IMU.magneticFieldAvailable())
    IMU.readMagneticField(m[0], m[1], m[2]);
}

/*------------------------------------------------------------------------------*/
/*                          ADC functions                                       */
/*------------------------------------------------------------------------------*/

extern "C" void SAADC_IRQHandler_v( void )          //can be used to perform operations
{                                                   //each time a conversion is complete
  if ( NRF_SAADC->EVENTS_END != 0 )
  {
    NRF_SAADC->EVENTS_END = 0;
  }
}

void initADC()
{
  nrf_saadc_disable();

  NRF_SAADC->RESOLUTION = NRF_SAADC_RESOLUTION_12BIT;

  // see Datasheet page 578 Pin assignments and Arduino Nano 33 BLE Pin Diagram
  // P0.04 - AIN2 -> Pin A0
  // P0.05 - AIN3 -> Pin A1
  NRF_SAADC->CH[0].CONFIG = ( SAADC_CH_CONFIG_GAIN_Gain1_4    << SAADC_CH_CONFIG_GAIN_Pos ) |
                            ( SAADC_CH_CONFIG_MODE_Diff       << SAADC_CH_CONFIG_MODE_Pos ) |
                            ( SAADC_CH_CONFIG_REFSEL_VDD1_4   << SAADC_CH_CONFIG_REFSEL_Pos ) |
                            ( SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos ) |
                            ( SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos ) |
                            ( SAADC_CH_CONFIG_TACQ_3us        << SAADC_CH_CONFIG_TACQ_Pos );

  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput2 << SAADC_CH_PSELP_PSELP_Pos;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_AnalogInput3 << SAADC_CH_PSELN_PSELN_Pos;

  // P0.30 - AIN6 -> Pin A2
  // P0.29 - AIN5 -> Pin A3
  NRF_SAADC->CH[1].CONFIG = ( SAADC_CH_CONFIG_GAIN_Gain1_4    << SAADC_CH_CONFIG_GAIN_Pos ) |
                            ( SAADC_CH_CONFIG_MODE_Diff       << SAADC_CH_CONFIG_MODE_Pos ) |
                            ( SAADC_CH_CONFIG_REFSEL_VDD1_4   << SAADC_CH_CONFIG_REFSEL_Pos ) |
                            ( SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos ) |
                            ( SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos ) |
                            ( SAADC_CH_CONFIG_TACQ_3us        << SAADC_CH_CONFIG_TACQ_Pos );

  NRF_SAADC->CH[1].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput6 << SAADC_CH_PSELP_PSELP_Pos;
  NRF_SAADC->CH[1].PSELN = SAADC_CH_PSELN_PSELN_AnalogInput5 << SAADC_CH_PSELN_PSELN_Pos;

  // P0.28 - AIN4 -> Pin A6
  // P0.03 - AIN1 -> Pin A7
  NRF_SAADC->CH[2].CONFIG = ( SAADC_CH_CONFIG_GAIN_Gain1_4    << SAADC_CH_CONFIG_GAIN_Pos ) |
                            ( SAADC_CH_CONFIG_MODE_Diff       << SAADC_CH_CONFIG_MODE_Pos ) |
                            ( SAADC_CH_CONFIG_REFSEL_VDD1_4   << SAADC_CH_CONFIG_REFSEL_Pos ) |
                            ( SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos ) |
                            ( SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos ) |
                            ( SAADC_CH_CONFIG_TACQ_3us        << SAADC_CH_CONFIG_TACQ_Pos );

  NRF_SAADC->CH[2].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput4 << SAADC_CH_PSELP_PSELP_Pos;
  NRF_SAADC->CH[2].PSELN = SAADC_CH_PSELN_PSELN_AnalogInput1 << SAADC_CH_PSELN_PSELN_Pos;


  NRF_SAADC->RESULT.MAXCNT = ADC_BUFFER_SIZE;
  NRF_SAADC->RESULT.PTR = ( uint32_t )&adcBuffer;

  NRF_SAADC->EVENTS_END = 0;
  nrf_saadc_int_enable( NRF_SAADC_INT_END );
  NVIC_SetPriority( SAADC_IRQn, 1UL );
  NVIC_EnableIRQ( SAADC_IRQn );

  nrf_saadc_enable();

  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;               //ADC conversion is temperature dependent
  while ( NRF_SAADC->EVENTS_CALIBRATEDONE == 0 );
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while ( NRF_SAADC->STATUS == ( SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos ) );
}


void initTimer4()
{
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  NRF_TIMER4->PRESCALER = 0;
  NRF_TIMER4->CC[0] = 16000000 / SAMPLES_PER_SECOND; // Needs prescaler set to 0 (1:1) 16MHz clock
  NRF_TIMER4->TASKS_START = 1;
}


void initPPI()
{
  NRF_PPI->CH[PPI_CHANNEL].EEP = ( uint32_t )&NRF_TIMER4->EVENTS_COMPARE[0];
  NRF_PPI->CH[PPI_CHANNEL].TEP = ( uint32_t )&NRF_SAADC->TASKS_START;
  NRF_PPI->FORK[PPI_CHANNEL].TEP = ( uint32_t )&NRF_SAADC->TASKS_SAMPLE;
  NRF_PPI->CHENSET = ( 1UL << PPI_CHANNEL );
}
