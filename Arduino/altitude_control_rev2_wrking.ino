// Full compliation of all the code used for altitude control

#define DEBUG
 
#define RX_IN_PIN 2 //pin 2 supports interrupts 
#define RX_OUT_PIN 5

#define MIN_THROTTLE 1050
#define MAX_THROTTLE 1950
#define START_RX_THROTTLE 1050

#define PID_RANGE 200  //must be even

//include libraries
#include <Servo.h> 
#include <PID_v1.h>
#include <SharpDistSensor.h>

//voltile flag is important for variables changed in ISR's
volatile unsigned long rx_raise_time = 0;
volatile unsigned int rx_throttle = START_RX_THROTTLE;

//state variables
double alt_cm = 0;
double alt_throttle = 1000;
double desired_alt_cm = 75;

//PID variables
double kp = 1;
double ki = 0.1;
double kd = 25;

// Analog pin to which the sensor is connected
const byte infrared_pin = A0;

// Window size of the median filter (odd number, 1 = no filtering)
const byte median_filter_size = 5;

/*
 * Polynomial fit curve coefficients C0 to C5 in relation:
 * Distance = C0 + C1*A + C2*A^2 + ... + C5*A^5
 * where A is the analog value read from the sensor
 * One coefficient minimum, six maximum (5th order polynomial)
 */
const float poly_coefficients[] = {346.333603654765, -3.90433660992245, 0.0223838067678020, -6.80489280702392e-05, 1.03861805361145e-07, -6.23685407343120e-11};
const byte n_coefficients = 6;  // Number of coefficients

/*
 * Minimum and maximum analog values for which to return a distance
 * These should represent a range of analog values within which the
 * polynomial fit curve is valid.
 */
const unsigned int min_val = 80; // ~150cm
const unsigned int max_val = 533; // ~20cm

Servo radioOut;

PID altPID(&alt_cm, &alt_throttle, &desired_alt_cm, kp, ki, kd, DIRECT);

// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor(infrared_pin, median_filter_size);

#ifdef DEBUG
long last_time = millis();
#endif



void setup() {
  //define the radio out pin as a servo
  radioOut.attach(RX_OUT_PIN);

  //define the radio input pin and ISR  
  attachInterrupt(digitalPinToInterrupt(RX_IN_PIN), radioIn, CHANGE);

  altPID.SetMode(AUTOMATIC);

  // Set custom polynomial fit curve coefficients and range
  sensor.setPolyFitCoeffs(n_coefficients, poly_coefficients, min_val, max_val);

  #ifdef DEBUG
  // initialize serial communication for debugging
  Serial.begin(115200);

  Serial.println();
  Serial.print("Setup OK, kp =  ");
  Serial.print(kp);
  Serial.print(", ki = ");
  Serial.print(ki);
  Serial.print(", kd = ");
  Serial.print(kd);
  Serial.print(", range = ");
  Serial.print(PID_RANGE);
  Serial.print(", median_filter_size = ");
  Serial.println(median_filter_size);
  #endif
}

void loop() {

  // Get distance from sensor
  alt_cm = alt_cm/2 + sensor.getDist()/2;

  // Limit the throttle based on radio input
  int min_throttle = max(MIN_THROTTLE, (rx_throttle - (PID_RANGE / 2)));
  int max_throttle = min(MAX_THROTTLE, max((min_throttle + 1), (rx_throttle + (PID_RANGE / 2))));

  // Cut throttle on failsafe switch
  if (rx_throttle < MIN_THROTTLE)
  {
    min_throttle = rx_throttle - 1;
    max_throttle = rx_throttle;
  }
  
  altPID.SetOutputLimits(min_throttle, max_throttle);

  // Iterate the PID controller
  altPID.Compute();

  // Pass the calculated throttle value to the flight controller
  radioOut.writeMicroseconds(alt_throttle);

  #ifdef DEBUG
  int time_elapsed = millis() - last_time;
  last_time = last_time + time_elapsed;
  // Print out variables for debug
  Serial.print("alt_cm: ");
  Serial.print(alt_cm);
  Serial.print("\trx_throttle: ");
  Serial.print(rx_throttle);
  Serial.print("\tmin_throttle: ");
  Serial.print(min_throttle);  
  Serial.print("\tmax_throttle: ");
  Serial.print(max_throttle);
  Serial.print("\talt_throttle: ");
  Serial.print(alt_throttle);
  Serial.print("\ttime_elapsed: ");
  Serial.println(time_elapsed);
  #endif
  
  delay(5);
  
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

