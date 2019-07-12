/* Code to synchronise the position of two props controlled using seperate ESCs.
 *  Uses interrupts from an optical sensor which determines when the beam 
 *  of light is broken (once every ~180 deg). The aim is to match the position
 *  so that the beams are both broken at the same time, thus both props are in 
 *  the same location. Position is adjusted through changing the PWM signal, 
 *  which effects rotational speed.   
 *  Date: 13/7/2019
 *  For: Illia at the University of Canterbury
 *  Author: Rhys Fitzgerald
 */

//------------------------------------------------------------------
//-----------------------WARNING: Incomplete------------------------
//------------------------------------------------------------------

#include <PWM.h>

#define CONST_ROTOR_PIN 2
#define VAR_ROTOR_PIN   3
#define PWM_MIN         4588
#define PWM_MAX         7864
#define MAX_DIFFERENCE  5000000 // = 5 Seconds in microseconds
#define ROTATION_ANGLE  180
#define INTEGRAL_TIME   10 //time in milliseconds
#define P_GAIN          30
#define I_GAIN          5

//pin definitions
uint8_t PWM_PIN = 9; //Using timer1 to avoid conflicts will mills() etc also knows as pin D9 on the board or OC1A
uint32_t FREQUENCY = 67;
uint16_t scalar = 4588;
int32_t time_diff = 0;
int32_t acc_error = 0;

volatile uint16_t const_period = 0;
volatile uint16_t prev_const_period = 0;
volatile uint16_t var_period = 0;
volatile uint16_t prev_var_period = 0;

uint16_t const_speed = 0;
uint16_t var_speed = 0;

int16_t speed_diff = 0;

void setup() {
  //---Pin initialisation---
  pinMode(PWM_PIN, INPUT);
  pinMode(CONST_ROTOR_PIN, INPUT);
  pinMode(VAR_ROTOR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(CONST_ROTOR_PIN), const_int, RISING);
  attachInterrupt(digitalPinToInterrupt(VAR_ROTOR_PIN), var_int, RISING);

//  Serial.begin(115200);
//  Serial.println();

  InitTimersSafe();
  SetPinFrequency(PWM_PIN, FREQUENCY);

  scalar = PWM_MIN; //initializing the PWM
  
  //can be applied to the timer (up to 16 bit). 1/2 of 65536 is 32768.
  pwmWriteHR(PWM_PIN, scalar);
}

void loop()
{
  //Time difference
  if ((const_period - var_period) < MAX_DIFFERENCE){
    time_diff = const_period - var_period;
  }

//  //Rotational speed <- Needed? Speed is proportial to the intterupt delay time?
//  const_speed = ROTATION_ANGLE / (const_period - prev_const_period);
//  var_speed = ROTATION_ANGLE / (var_period - prev_var_period);
//
//  speed_diff = const_speed - var_speed;

  acc_error += time_diff * (INTEGRAL_TIME/1000); //will this accumulate too fast?
  
  //Theoretically, PI control here...
  scalar = (time_diff * P_GAIN) + (time_diff / I_GAIN); //unsure if correct at this stage

  //Adjusting PWM depening on PI control
  pwmWriteHR(PWM_PIN, scalar);

  delay(INTEGRAL_TIME); // might need to change to delayMicrsoseconds(us);
}

//Time stamps when each position is detected (every half rotation)
void const_int()
{
  prev_const_period = const_period;
  const_period = micros();
}

void var_int()
{
  prev_var_period = var_period;
  var_period = micros();
}
