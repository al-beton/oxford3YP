//Demo of how the radio receiver interrupts work in isolation

#define RX_PIN 2

//voltile flag is important for variables changed in ISR's
volatile unsigned long ISR_raise_time = 0;
volatile unsigned int signal_time = 0;

void setup() {
  attachInterrupt(digitalPinToInterrupt(RX_PIN), radioIn, CHANGE);
  Serial.begin(9600);
}

void loop() {
  Serial.println(signal_time);
  delay(1);        // delay in between reads for stability
}

void radioIn()
{
  if(digitalRead(RX_PIN) == HIGH)
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

