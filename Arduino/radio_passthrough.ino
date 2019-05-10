//Demo of how the radio receiver signal can be passed by the arduino to the Quadcopter

#define RX_IN_PIN 2
#define RX_OUT_PIN 5

//include servo library used for PWM output
#include <Servo.h> 
Servo servoOut;

//voltile flag is important for variables changed in ISR's
volatile unsigned long ISR_raise_time = 0;
//initial value of 1000 to stop servo errors
volatile unsigned int signal_time = 1000;

void setup() {
  //define the servo pin
  servoOut.attach(RX_OUT_PIN);

  //define the radio input pin and ISR  
  attachInterrupt(digitalPinToInterrupt(RX_IN_PIN), radioIn, CHANGE);
  Serial.begin(9600);
}

void loop() {
  servoOut.writeMicroseconds(signal_time);
  Serial.println(signal_time);
  delay(20);        // delay in between reads for stability
}

void radioIn()
{
  if(digitalRead(RX_IN_PIN) == HIGH)
  {
    //start the timer on the rising edge of the pwm signal
    ISR_raise_time = micros();   
    
  }
  else
  {
    //if its not the rising edge it must be the falling edge, so stop the timer
    signal_time = micros() - ISR_raise_time;
  }
}

