#pragma once

//#include <iostream>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

class DriveManager {
    private:
    frc::Joystick *stick;
    frc::DifferentialDrive *drive;
    
    WPI_TalonSRX *leftMotor;
    WPI_TalonSRX *rightMotor;
    WPI_TalonSRX *rightFollowerMotor;
    WPI_TalonSRX *leftFollowerMotor;

    double zStickControl;
    double yStickControl;

    public:
    DriveManager();
    void driveControl();
};