//Demo of how the arduino can poll the ultrasonic range finder in a non-blocking way

#define TRIG_PIN 4
#define ECHO_PIN 3 //pin 3 supports interrupts 

#define PING_LIMIT 25000 //approximately 4m range, 40Hz 

//volatile flag is important for variables changed in ISR's
volatile unsigned long ping_start_time = 0;
volatile unsigned long ping_end_time = 0;
volatile unsigned int ping_time = 0;
unsigned int cm;

void setup() {
  //define the HC-SR04 trig pin as an output
  pinMode(TRIG_PIN, OUTPUT);
 
  //define the echo pin and ping ISR  
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), pingReturn, FALLING);

  // initialize serial communication for testing
  Serial.begin(9600);

}

void loop() {
  pingOut();
}

void pingOut() {
  if (ping_start_time + PING_LIMIT < micros())
  {
    Serial.println(microsecondsToCentimeters(ping_time));
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, LOW);
    ping_start_time = micros();
  }
}

void pingReturn()
{
  ping_end_time = micros();
  ping_time = ping_end_time - ping_start_time;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

