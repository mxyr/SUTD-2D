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

#define Kp 20
#define Kd 25
#define Ki 0

#define leftMotorBaseSpeed 65 //tune
#define rightMotorBaseSpeed 65

#define maxSpeed 100
#define minSpeed 10

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];
int xiao = 450;
double weightage[SensorCount] = {40, 20, -20, -40}; //change accordinlgly, how much u want it to turn
double error = 0;
double errorSum = 0;
double lastError = 0;

int leftMotorSpeed;
int rightMotorSpeed;
int csensor[4];
int leftPrevious;
int rightPrevious;
float yz_gm = 50; //光敏电阻触发阈值（电压）
int pin_gm = A0; //光敏接口针脚
int ini_gm = 30;
int zt = 0; //状态数，初始为0（关闭）
void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    8, 9, 10, 11
  }, SensorCount); // change pins
  //qtr.setEmitterPin(2);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (4, OUTPUT);
  pinMode (5, OUTPUT);
  pinMode (6, OUTPUT);
  pinMode (7, OUTPUT);
  pinMode(pin_gm, INPUT);

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
}

void loop()
{
  if (zt == 0) {
    delay(1000);
    float gm = analogRead(pin_gm); //读取光敏针脚
    if (gm < yz_gm) { //有额外照明(gm<yz_gm)（电阻小，电压低）
      zt = 1;
    }
  }
  else if (zt == 1) {
    qtr.read(sensorValues);
    for (int i = 0; i < SensorCount; i++)
    {
      Serial.print("Sensor");
      Serial.print(i);
      Serial.print (": ");
      Serial.println(sensorValues[i]);
      if (sensorValues[i] < xiao) {
        sensorValues[i] = 0;
      }
      else {
        sensorValues[i] = ((long)1000) * (sensorValues[i] - xiao) / (2500 - xiao);
      }
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
}

double weightedAverage (void)
{
  int sum = 0;
  double weightedSum = 0;
  for (int i = 0; i < SensorCount; i++)
  {
    weightedSum += sensorValues[i] * weightage[i];
    sum += sensorValues[i];
    Serial.print("sensorValues");
    Serial.print(i);
    Serial.print (": ");
    Serial.println(sensorValues[i]);
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
