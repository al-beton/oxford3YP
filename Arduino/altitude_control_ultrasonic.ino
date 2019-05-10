// Full compliation of all the code used for altitude control

#define RX_IN_PIN 2 //pins 2 and 3 support interrupts 
#define ECHO_PIN 3
#define TRIG_PIN 4
#define RX_OUT_PIN 5

#define PING_LIMIT 25000 //approximately 8m range, 20Hz 
#define MIN_THROTTLE 1000
#define START_RX_THROTTLE 1050

//include libraries
#include <Servo.h> 
#include <PID_v1.h>

//voltile flag is important for variables changed in ISR's
volatile unsigned long rx_raise_time = 0;
volatile unsigned int rx_throttle = START_RX_THROTTLE;
volatile unsigned long ping_start_time = 0;
volatile unsigned long ping_end_time = 0;
volatile unsigned int ping_time = 0;
volatile bool ping_ready = 1;

double alt_cm = 0;
double alt_throttle = 1000;
double desired_alt_cm = 40;

double kp = 1;
double ki = 1;
double kd = 1;

Servo servoOut;

PID altPID(&alt_cm, &alt_throttle, &desired_alt_cm, kp, ki, kd, DIRECT);

void setup() {
  //define the radio out pin as a servo
  servoOut.attach(RX_OUT_PIN);
  //define the HC-SR04 trig pin as an output
  pinMode(TRIG_PIN, OUTPUT);

  //define the radio input pin and ISR  
  attachInterrupt(digitalPinToInterrupt(RX_IN_PIN), radioIn, CHANGE);
  //define the echo pin and ping ISR  
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), pingReturn, FALLING);

  altPID.SetMode(AUTOMATIC);

  // initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  alt_cm = microsecondsToCentimeters(ping_time);
  altPID.SetOutputLimits(MIN_THROTTLE, rx_throttle);
  altPID.Compute();
  Serial.print("alt_cm: ");
  Serial.print(alt_cm);
  Serial.print("\talt_throttle: ");
  Serial.print(alt_throttle);
  Serial.print("\trx_throttle: ");
  Serial.println(rx_throttle);
  servoOut.writeMicroseconds(alt_throttle);
  pingOut();
  delay(100);
  
}

void radioIn() {
  if(digitalRead(RX_IN_PIN) == HIGH)
  {
    //start the timer on the rising edge of the pwm signal
    rx_raise_time = micros();   
  }
  else
  {
    //if its not the rising edge it must be the falling edge, so stop the timer
    rx_throttle = micros() - rx_raise_time;
  }
}

void pingOut() {
  if (ping_start_time + PING_LIMIT < micros())
  {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, LOW);
    ping_start_time = micros();
  }
}

void pingReturn() {
  ping_end_time = micros();
  ping_time = ping_end_time - ping_start_time;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
