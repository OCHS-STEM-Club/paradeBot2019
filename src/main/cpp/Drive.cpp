#include "Drive.hpp"

DriveManager::DriveManager() {
 xbox = new frc::XboxController(1);

/*  leftMotor = new WPI_TalonSRX(2);
    rightMotor = new WPI_TalonSRX(3);
    leftFollowerMotor = new WPI_TalonSRX(4);
    rightFollowerMotor = new WPI_TalonSRX(5); */

    double kTimeoutMs = 10;
	double kPIDLoopIdx = 0;

    pidMotor = new WPI_TalonSRX(3);
    pidMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    //pidMotor->SetSensorPhase(true); //to invert sensor
	//pidMotor->SetInverted(false);
	pidMotor->ConfigAllowableClosedloopError(kPIDLoopIdx, 0, kTimeoutMs);

    pidMotor->ConfigNominalOutputForward(0, kTimeoutMs);
	pidMotor->ConfigNominalOutputReverse(0, kTimeoutMs);
	pidMotor->ConfigPeakOutputForward(1, kTimeoutMs);
	pidMotor->ConfigPeakOutputReverse(-1 , kTimeoutMs);

	/* set closed loop gains in slot0 */
	pidMotor->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
	pidMotor->Config_kP(kPIDLoopIdx, 16, kTimeoutMs);
	pidMotor->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	pidMotor->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

    leftMotor = new frc::Victor{1};
    rightMotor = new frc::Victor{2};
    

    //leftMotor = new WPI_VictorSPX(1);
    //rightMotor = new WPI_VictorSPX(2);
 //   leftFollowerMotor = new WPI_VictorSPX(4);
 //   rightFollowerMotor = new WPI_VictorSPX(5);

    drive = new frc::DifferentialDrive(*leftMotor,*rightMotor);

  //  leftFollowerMotor->Follow(*leftMotor);
  //  rightFollowerMotor->Follow(*rightMotor);

 
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
    yXboxControl = xbox->GetRawAxis(1);//deadband(xboxDrive->GetRawAxis(1), 0.1);
    xXboxControl = xbox->GetRawAxis(4);//deadband(xboxDrive->GetRawAxis(4), 0.1);

    

    if (!xbox->GetRawButton(6)) {
        yXboxControl = yXboxControl * 0.75;
        xXboxControl = xXboxControl * 0.75;
    }
    //pidMotor->Set(0.4);

    pidMotor->Set(ControlMode::Position, 0); //replace 0 with enc rotations
    pidMotor->Set(ControlMode::Velocity, 0); //velocity in native units / 100ms
    

    drive->ArcadeDrive(-yXboxControl, xXboxControl);

    //leftMotor->Set(xboxDrive->GetRawAxis(1) * 0.2);
}




/*
void DriveManager::PID_Initialize (int Kp, int Ki, int Kd, int imax, int Kp_value, int Ki_value, int Kd_value, int imax_value)
{
//intialize ze values of ze pid structair
Kp = Kp_value;
Ki = Ki_value;
Kd = Kd_value;
imax = imax_value;
}


unsigned char PID (int Kp, int Ki, int Kd, int last_error, int error_sum, int imax, int error)
{
int P;
int I;
int D;

P = (((long)error * (Kp))/ 1000);
I = (((long)(error_sum) * (Ki)) / 10000);
D = (((long)(error - (last_error)) * (Kd)) / 10);

last_error = error;

//if(!disabled_mode) {
//    error_sum += error;
//}

if (I > imax) {
	error_sum = imax;
}
else if (I < -imax) {
	error_sum = -imax;
}


//return Limit_Mix(2000 + 132 + P + I - D);
return (2000 + 132 + P + I - D);


} */