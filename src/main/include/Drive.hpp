#pragma once

//#include <iostream>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

class DriveManager {
    private:
    //frc::Joystick *stick;
    frc::XboxController *xbox;
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

    int Kp; // In tenths
    int Ki; // In thousandths
    int Kd; // In tenths
    int last_error;
    int error_sum;
    int imax;

    public:
    DriveManager();
    void driveControl();
    //void PID_Initialize (int Kp, int Ki, int Kd, int imax, int Kp_value, int Ki_value, int Kd_value, int imax_value);
    //unsigned char PID (int Kp, int Ki, int Kd, int last_error, int error_sum, int imax, int error);
    
   /* typedef struct {
        int Kp; // In tenths
        int Ki; // In thousandths
        int Kd; // In tenths
        int last_error;
        int error_sum;
        int imax;
    } PID_STRUCT;
    


void PID_Initialize (PID_STRUCT* pid_info, int Kp_value, int Ki_value, int Kd_value, int imax_value);
unsigned char PID (PID_STRUCT* pid_info, int error);*/



    //void PID_Initialize (int Kp, int Ki, int Kd, int imax, int Kp_value, int Ki_value, int Kd_value, int imax_value);
    //unsigned char PID (int Kp, int Ki, int Kd, int last_error, int error_sum, int imax, int error);
};