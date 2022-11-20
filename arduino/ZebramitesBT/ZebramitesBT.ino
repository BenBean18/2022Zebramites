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
  bluetooth.setTimeout(5);
  Serial.begin(115200);
  leftMotor.setInverted(false);
  rightMotor.setInverted(false);
  intake.setInverted(false);

  RSL::initialize();
  RSL::setState(RSL_ENABLED);
}

void loop() {
  // format: z<MOTOR ID>;<MOTOR POWER>;
  while(bluetooth.available() > 0 && bluetooth.read() == 'z')
  {
    Serial.println("data");
    int motorId = bluetooth.readStringUntil(';').toInt();
    double speed = bluetooth.readStringUntil(';').toFloat();

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
    Serial.print(motorId);
    Serial.print(" ");
    Serial.println(speed);
    RSL::update();
  }
  RSL::update();
  delay(1);
}