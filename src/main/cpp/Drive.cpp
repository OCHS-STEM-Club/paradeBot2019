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





void PID_Initialize (PID_STRUCT* pid_info, int Kp_value, int Ki_value, int Kd_value, int imax_value)
{
//intialize ze values of ze pid structair
pid_info->Kp = Kp_value;
pid_info->Ki = Ki_value;
pid_info->Kd = Kd_value;
pid_info->imax = imax_value;
}


unsigned char PID (PID_STRUCT* pid_info, int error)
{
int P;
int I;
int D;

P = (((long)error * (pid_info->Kp))/ 1000);
I = (((long)(pid_info->error_sum) * (pid_info->Ki)) / 10000);
D = (((long)(error - (pid_info->last_error)) * (pid_info->Kd)) / 10);

pid_info->last_error = error;

if(!disabled_mode)
pid_info->error_sum += error;

if (I > pid_info->imax)
	pid_info->error_sum = pid_info->imax;
else if (I < -pid_info->imax)
	pid_info->error_sum = -pid_info->imax;


return Limit_Mix(2000 + 132 + P + I - D);
}