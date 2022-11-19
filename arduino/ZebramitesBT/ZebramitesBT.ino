/**
 * <insert header comment here>
 */

#include <Alfredo_NoU2.h>
#include <BluetoothSerial.h>

BluetoothSerial bluetooth;

// If your robot has more than a drivetrain and one servo, add those actuators here 
NoU_Motor leftMotor(3);
NoU_Motor rightMotor(4);
NoU_Motor intake(1);



void setup() {
  bluetooth.begin("Zebramites");
  leftMotor.setInverted(false);
  rightMotor.setInverted(false);
  intake.setInverted(false);

  RSL::initialize();
  RSL::setState(RSL_ENABLED);
}

void loop() {
  if(bluetooth.available() > 0)
  {
    int motorId = bluetooth.parseInt();
    double speed = bluetooth.parseFloat();

    switch (motorId) {
      case 1:
        intake.set(speed);
        break;
      case 3:
        leftMotor.set(speed);
        break;
      case 4:
        rightMotor.set(speed);
        break;
    }
  }
  RSL::update();
  delay(10);
}