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
#define MOTORRESISTANCE 5.6

//#define POLEPAIRS 14
#define POLEPAIRS 11
#define CLOSEDLOOP true

#define MESSAGESIZE 8
#define MAXTARGETVELOCITY 50
#define MAXTARGETCURRENT 10
#define MAXTARGETVOLTAGE 6

BLDCMotor motor1 = BLDCMotor(POLEPAIRS);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PC0, PC1, PC2, PC13);
MagneticSensorPWM sensor1 = MagneticSensorPWM(SENSORPWM1, 7, 935);

BLDCMotor motor2 = BLDCMotor(POLEPAIRS);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1, PA2, PC14);
MagneticSensorPWM sensor2 = MagneticSensorPWM(SENSORPWM2, 7, 935);

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
  if (CLOSEDLOOP) {
    // main FOC algorithm function
    motor1.loopFOC();
    motor2.loopFOC();

    // Motion control function
    motor1.move(motorTargetVoltage);
    motor2.move(motorTargetVoltage);
  } else {
    motor1.move(motorTargetVelocity);
    motor2.move(motorTargetVelocity);
  }
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

  if (CLOSEDLOOP) {
    sensor1.init();
    motor1.linkSensor(&sensor1);
  }

  driver1.voltage_power_supply = SUPPLYVOLTAGE;
  driver1.voltage_limit = VOLTAGELIMITDRIVER;
  driver1.init();
  motor1.linkDriver(&driver1);

  motor1.voltage_limit = VOLTAGELIMITMOTOR;
  //motor1.phase_resistance = MOTORRESISTANCE;
  motor1.voltage_sensor_align = VOLTAGELIMITMOTOR;

  if (CLOSEDLOOP) {
    motor1.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::torque;
    motor1.init();
    motor1.initFOC();
  } else {
    motor1.controller = MotionControlType::velocity_openloop;
    motor1.init();
  }

  if (CLOSEDLOOP) {
    sensor2.init();
    motor2.linkSensor(&sensor2);
  }

  driver2.voltage_power_supply = SUPPLYVOLTAGE;
  driver2.voltage_limit = VOLTAGELIMITDRIVER;
  driver2.init();
  motor2.linkDriver(&driver2);

  motor2.voltage_limit = VOLTAGELIMITMOTOR;
  //motor2.phase_resistance = MOTORRESISTANCE;
  motor2.voltage_sensor_align = VOLTAGELIMITMOTOR;

  if (CLOSEDLOOP) {
    motor2.torque_controller = TorqueControlType::voltage;
    motor2.controller = MotionControlType::torque;
    motor2.init();
    motor2.initFOC();
  } else {
    motor2.controller = MotionControlType::velocity_openloop;
    motor2.init();
  }
}

void receiveFun (int bytes)
{
  int i = 0;
  while (Wire.available() && i < MESSAGESIZE)
  {
    message[i] = Wire.read();
    i++;
  }
  /*
    if(message[0] != 0xFF) {
    blink(3,100);
    return;
    }
  */
  float in = message[1];
  in = in - 128;


  if (CLOSEDLOOP) {
    in = in / 128 * MAXTARGETVOLTAGE;
    motorTargetVoltage = in;
  } else {
    in = in / 128 * MAXTARGETVELOCITY;
    motorTargetVelocity = in;
  }
  /*
    char rec = Wire.read();
    message[i] = rec;
    motorTargetVelocity = rec;
    i++;
  */
}

void requestFun()
{
  Wire.write(message);
}
