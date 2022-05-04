#include <Wire.h>
#include <SimpleFOC.h>

/*
   Rotary Encoder Wires:
   Purple - PWM -  Gray
   Green  - GND -  White
   Blue   - 5V  -  Black
*/

#define SENSORPWM1 PB6
#define SENSORPWM2 PB7

#define LEDPIN PA7
#define SLAVEADDRESS 0x8
#define SDAPIN PB9
#define SCLPIN PB8

#define SUPPLYVOLTAGE 8.4
#define VOLTAGELIMITDRIVER 6
#define VOLTAGELIMITMOTOR 3
#define MOTORRESISTANCE 5.57

#define SENSORMINPULSE 7
#define SENSORMAXPULSE 935

#define POLEPAIRS 11
#define CLOSEDLOOP true

#define MESSAGESIZE 8
#define MAXTARGETVELOCITY 50
#define MAXTARGETCURRENT 10
#define MAXTARGETVOLTAGE 6

BLDCMotor motor1 = BLDCMotor(POLEPAIRS, MOTORRESISTANCE);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PC0, PC1, PC2, PC13);
MagneticSensorPWM sensor1 = MagneticSensorPWM(SENSORPWM1, SENSORMINPULSE, SENSORMAXPULSE);

BLDCMotor motor2 = BLDCMotor(POLEPAIRS, MOTORRESISTANCE);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1, PA2, PC14);
MagneticSensorPWM sensor2 = MagneticSensorPWM(SENSORPWM2, SENSORMINPULSE, SENSORMAXPULSE);

char message[MESSAGESIZE];
float motorTargetCurrent = 0;
float motorTargetVelocity = 0;
float motorTargetVoltage = 0;

void setup()
{
  blink(2, 100);

  initI2C();
  initMotors();

  blink(3, 200);
}

void loop()
{
  // main FOC algorithm function
  motor1.loopFOC();
  motor2.loopFOC();

  // Motion control function
  motor1.move(motorTargetVoltage);
  motor2.move(motorTargetVoltage);
}

void blink(int amount, int del) {
  pinMode(LEDPIN, OUTPUT);

  for (int i = 0; i < amount; i++) {
    digitalWrite(LEDPIN, HIGH);
    delay(del);
    digitalWrite(LEDPIN, LOW);
    delay(del);
  }
}

void initI2C() {
  Wire.setSDA(SDAPIN);
  Wire.setSCL(SCLPIN);
  Wire.begin(SLAVEADDRESS);
  Wire.onReceive(receiveFun);
  Wire.onRequest(requestFun);
}

void initMotors() {
  pinMode(SENSORPWM1, INPUT);
  pinMode(SENSORPWM2, INPUT);

  sensor1.init();
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = SUPPLYVOLTAGE;
  driver1.voltage_limit = VOLTAGELIMITDRIVER;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = VOLTAGELIMITMOTOR;
  motor1.voltage_sensor_align = VOLTAGELIMITMOTOR;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor1.init();
  motor1.initFOC();

  sensor2.init();
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = SUPPLYVOLTAGE;
  driver2.voltage_limit = VOLTAGELIMITDRIVER;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = VOLTAGELIMITMOTOR;
  motor2.voltage_sensor_align = VOLTAGELIMITMOTOR;
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.controller = MotionControlType::torque;
  motor2.init();
  motor2.initFOC();
}

void receiveFun (int bytes)
{
  int i = 0;
  while (Wire.available() && i < MESSAGESIZE)
  {
    message[i] = Wire.read();
    i++;
  }

  float spd = message[1];
  float str = message[2];

  //de-normalize values
  spd = spd - 128;
  spd = spd / 128 * MAXTARGETVOLTAGE;
  motorTargetVoltage = spd;
}

void requestFun()
{
  // Convert from angular rad/s to linear velocity m/s
  // TODO calculate correct conversion factor
  float vel = sensor1.getVelocity() + sensor2.getVelocity() / 2 * WHEELRADIUS;
  // TODO check I2C float specs, convert if neccessary
  Wire.write(vel);
}
