#pragma once

//#include <iostream>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
 
class DriveManager {
    private:
    //frc::Joystick *stick;
    frc::XboxController *xbox;
    frc::Joystick *stick;
    frc::DifferentialDrive *drive;
    
/*  WPI_TalonSRX *leftMotor;
    WPI_TalonSRX *rightMotor;
    WPI_TalonSRX *rightFollowerMotor;
    WPI_TalonSRX *leftFollowerMotor; */

    WPI_TalonSRX *pidMotor;

    frc::Victor *leftMotor;
    frc::Victor *rightMotor;
//    WPI_VictorSPX *rightFollowerMotor;
//    WPI_VictorSPX *leftFollowerMotor;


    double xXboxControl;
    double yXboxControl;

    double xStickControl;

    double velocityOut;


    public:
    DriveManager();
    void driveControl();
    void pidControl();
};