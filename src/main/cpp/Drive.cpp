#include "Drive.hpp"

DriveManager::DriveManager() {
 xbox = new frc::XboxController(1);
 stick = new frc::Joystick(0);

/*  leftMotor = new WPI_TalonSRX(2);
    rightMotor = new WPI_TalonSRX(3);
    leftFollowerMotor = new WPI_TalonSRX(4);
    rightFollowerMotor = new WPI_TalonSRX(5); */

    double kTimeoutMs = 10;
	double kPIDLoopIdx = 0;

    pidMotor = new WPI_TalonSRX(2);
    pidMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    //pidMotor->SetSensorPhase(true); //to invert sensor
	//pidMotor->SetInverted(false);
	pidMotor->ConfigAllowableClosedloopError(kPIDLoopIdx, 0, kTimeoutMs);

    //collins stuff, do not know purpose
    /*pidMotor->ConfigNominalOutputForward(0, kTimeoutMs);
	pidMotor->ConfigNominalOutputReverse(0, kTimeoutMs);
	pidMotor->ConfigPeakOutputForward(1, kTimeoutMs);
	pidMotor->ConfigPeakOutputReverse(-1 , kTimeoutMs); */

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

double velocityConverter(double input) { //converts rotations per minute to native units per 100 ms
    //return (input * 4096) * 60 / 1000.0; //10 is for time conversion. 4096 is units per rotation for encoder
    return input * 6.8266666667; //ratio of rpm to na/100ms
}

void DriveManager::driveControl() {
    yXboxControl = xbox->GetRawAxis(1);//deadband(xboxDrive->GetRawAxis(1), 0.1);
    xXboxControl = xbox->GetRawAxis(4);//deadband(xboxDrive->GetRawAxis(4), 0.1);


    if (!xbox->GetRawButton(6)) {
        yXboxControl = yXboxControl * 0.75;
        xXboxControl = xXboxControl * 0.75;
    }
    
    drive->ArcadeDrive(-yXboxControl, xXboxControl);

    //leftMotor->Set(xboxDrive->GetRawAxis(1) * 0.2);

}

void DriveManager::pidControl() {
    xStickControl = deadband(stick->GetRawAxis(1), 0.1);
    velocityOut = xStickControl * 1; //replace 1 with max velocity (rot/min)

    //pidMotor->Set(ControlMode::Position, 0); //replace 0 with enc rotations
    pidMotor->Set(ControlMode::Velocity, velocityConverter(velocityOut)); //velocity in native units / 100ms
    //pidMotor->Set(xStickControl);

    frc::SmartDashboard::PutNumber("velocity", pidMotor->GetSensorCollection().GetQuadratureVelocity());
}