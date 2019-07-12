#include <PWM.h>

#define PWM_IN      3
#define CONST_ROTOR 1
#define VAR_ROTOR   2
#define PWM_MIN     4588
#define PWM_MAX     7864

//pin definitions
uint8_t PWM_PIN = 9; //Using timer1 to avoid conflicts will mills() etc also knows as pin D9 on the board or OC1A
uint32_t FREQUENCY = 67;
uint16_t scalar = 4588;

void setup() {
  //pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_IN, INPUT);
  pinMode(CONST_ROTOR, INPUT);
  pinMode(CONST_ROTOR, INPUT);

  InitTimersSafe();
  Serial.begin(115200);
  Serial.println();
  SetPinFrequency(PWM_PIN, FREQUENCY);

  scalar = PWM_MAX;
  
  //can be applied to the timer (up to 16 bit). 1/2 of 65536 is 32768.
  pwmWriteHR(PWM_PIN, scalar);
}

void loop()
{
  
}
