// #include <SimpleFOC.h>

// // BLDC motor instance BLDCMotor(polepairs, (R), (KV))
// BLDCMotor motor = BLDCMotor(7, 0.27, 360);

// // BLDC driver instance BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, (en))
// BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15);


// // inline current sense instance InlineCurrentSense(R, gain, phA, phB, phC)
// InlineCurrentSense currentsense = InlineCurrentSense(0.003, -64.0/7.0, PA2, PA6, PB1);
#define HAL_ADC_MODULE_ONLY
#define HAL_OPAMP_MODULE_ENABLED

#include <SimpleFOC.h>

// BLDC motor instance BLDCMotor(polepairs, (R), (KV))
BLDCMotor motor = BLDCMotor(7, 0.27, 360);

// BLDC driver instance BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, (en))
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15);


// position / angle sensor instance
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// inline current sense instance InlineCurrentSense(R, gain, phA, phB, phC)
//InlineCurrentSense currentsense = InlineCurrentSense(0.003, -64.0/7.0,PA2, PA6, PB1);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003, -64.0/7.0,PA2, PA6, PB1);


// commander instance
Commander command = Commander(Serial);
void doTarget(char* cmd){command.motion(&motor, cmd);}

void setup() {
    // start serial
    Serial.begin(115200);

    // set I2C clock speed
    Wire.setClock(400000);

    // initialize sensor
    sensor.init();
    // link sensor to motor
    motor.linkSensor(&sensor);

    // set power supply voltage
    driver.voltage_power_supply = 14.8;
    // set driver voltage limit, this phase voltage
    driver.voltage_limit = 14;
    // initialize driver
    driver.init();
    // link driver to motor
    motor.linkDriver(&driver);

    // link driver to current sense
    currentsense.linkDriver(&driver);


    // set motion control type to velocity
    motor.controller = MotionControlType::velocity;

    // set torque control type to FOC current
    motor.torque_controller = TorqueControlType::foc_current;

    // set FOC modulation type to space vector modulation
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // velocity PID controller
    motor.PID_velocity.P = 0.8;
    motor.PID_velocity.I = 0.01;
    motor.PID_velocity.D = 0;
    // set motor voltage limit, this limits Vq
    motor.voltage_limit = 20;

    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.03;

    // use monitoring
    motor.useMonitoring(Serial);

    // initialize motor
    motor.init();

    // initialize current sensing and link it to the motor
    // https://docs.simplefoc.com/inline_current_sense#where-to-place-the-current_sense-configuration-in-your-foc-code
    currentsense.init();
    motor.linkCurrentSense(&currentsense);

    // align sensor and start FOC
    motor.initFOC();

    // add command to commander
    command.add('M', doTarget, "target");

    _delay(1000);
}

void loop() {
    // main FOC algorithm function
    // the faster you run this function the better
    motor.loopFOC();

    // this function can be run at much lower frequency than loopFOC()
    motor.move();

    // significantly slowing the execution down
    motor.monitor();

    // user communication
    command.run();
}
