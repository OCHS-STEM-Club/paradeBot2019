#include "Drive.hpp"

DriveManager::DriveManager() {
    stick = new frc::Joystick(0);

    leftMotor = new WPI_TalonSRX(1);
    rightMotor = new WPI_TalonSRX(2);
    leftFollowerMotor = new WPI_TalonSRX(3);
    rightFollowerMotor = new WPI_TalonSRX(4);

    drive = new frc::DifferentialDrive(*leftMotor,*rightMotor);

    leftFollowerMotor->Follow(*leftMotor);
    rightFollowerMotor->Follow(*rightMotor);
 
}

int Sign(double input) {
    if (input > 0) {
        return 1;
    }
    else if (input < 0) {
        return -1;
    }
    else if (input == 0) {
        return 0;
    }
}

double deadband(double joystickValue, double deadbandValue) {
    if(abs(joystickValue) < 0.2){
        return 0;
    }
    else{
        return (1 / (1 - deadbandValue)) * (joystickValue + (-Sign(joystickValue) * deadbandValue));
    } 
}

void DriveManager::driveControl() {
    yStickControl = deadband(stick->GetRawAxis(1), 0.1);
    zStickControl = deadband(stick->GetRawAxis(2), 0.1);

    drive->ArcadeDrive(yStickControl, zStickControl);
}