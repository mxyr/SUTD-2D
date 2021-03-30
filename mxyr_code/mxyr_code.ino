#include <QTRSensors.h>

// This example is designed for use with six analog QTR sensors. These
// reflectance sensors should be connected to analog pins A0 to A5. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is. Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 5000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

#define Kp 1.8
#define Kd 0 // smth 0.4 to 0.9
#define Ki 0

#define leftMotorBaseSpeed 70 //tune
#define rightMotorBaseSpeed 70

#define maxSpeed 120
#define minSpeed 10

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

double weightage[SensorCount] = {260.625, 200.875, -200.875, -260.625}; //change accordinlgly, how much u want it to turn
double error = 0;
double errorSum = 0;
double lastError = 0;

int leftMotorSpeed;
int rightMotorSpeed;

int leftPrevious;
int rightPrevious;

void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    8, 9, 10, 11
  }, SensorCount); // change pins
  //qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (4, OUTPUT);
  pinMode (5, OUTPUT);
  pinMode (6, OUTPUT);
  pinMode (7, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  //LDR while loop

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  /* for (uint8_t i = 0; i < SensorCount; i++)
    {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++)
    {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);
  */
}

void loop()
{
  qtr.readCalibrated(sensorValues);

  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print("Sensor");
    Serial.print(i);
    Serial.print (": ");
    Serial.println(sensorValues[i]);
  }

  error = weightedAverage();

  int calibration = (int)round(PID(error));

  Serial.print ("Calibration: ");
  Serial.println (calibration);

  leftMotorSpeed = leftMotorBaseSpeed - calibration;
  rightMotorSpeed = rightMotorBaseSpeed + calibration;
  Serial.print(leftMotorSpeed);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  Serial.println();

  //white readings
  // if (allwhite())
  // leftmotorspeed = leftprevious;

  // pseudocode for allwhite (bool)
  // int sum]
  // for (i 0 i < 5 i ++ )
  //      sum += sensorValues[i];
  // if sum < <something>


  //normal

  if (leftMotorSpeed >= 0 and rightMotorSpeed >= 0)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

    analogWrite (5, rightMotorSpeed);
    digitalWrite (4, LOW); //LOW is forward

    analogWrite (6, leftMotorSpeed);
    digitalWrite (7, HIGH);
  }
  else if (leftMotorSpeed < 0 and rightMotorSpeed > 0)
  {
    leftMotorSpeed = constrain(0 - leftMotorSpeed, 0, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

    analogWrite (5, rightMotorSpeed);
    digitalWrite (4, LOW);

    analogWrite (6, leftMotorSpeed);
    digitalWrite (7, LOW);
  }
  else if (leftMotorSpeed > 0 and rightMotorSpeed < 0)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
    rightMotorSpeed = constrain(0 - rightMotorSpeed, 0, maxSpeed);

    analogWrite (5, rightMotorSpeed);
    digitalWrite (4, HIGH);

    analogWrite (6, leftMotorSpeed);
    digitalWrite (7, HIGH);


  }
  Serial.print(leftMotorSpeed);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  Serial.println();
  leftPrevious = leftMotorSpeed;
  rightPrevious = rightMotorSpeed;

}

double weightedAverage (void)
{
  int sum = 0;
  double weightedSum = 0;
  for (int i = 0; i < SensorCount; i++)
  {
    weightedSum += sensorValues[i] * weightage[i];
    sum += sensorValues[i];
  }

  double weightedAverage = weightedSum / (double) sum;

  Serial.print ("Weighted Average: ");
  Serial.println (weightedAverage);

  return weightedAverage;
}

double PID (int error)
{
  errorSum += error;
  double differentialError = error - lastError;
  lastError = error;

  double output = Kp * error + Kd * differentialError + Ki * errorSum;

  return output;
}
