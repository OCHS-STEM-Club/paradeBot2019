#include "Pneumatics.hpp" 

PneumaticManager::PneumaticManager () {
    compressor = new frc::Compressor(0);
    compressor->SetClosedLoopControl(true);
    //compressor->Stop();

    solonoid = new frc::Solenoid(1);
 
    stick = new frc::Joystick(0);

    rotateLatch = false;

    //driveMotorLeft->GetSensorCollection().SetQuadraturePosition(0, 10);
}

void PneumaticManager::air() {
    if (stick->GetRawButton(4)) {
        solonoid->Set(true);
    }
    else {
        solonoid->Set(false);
    }

    //*encRotLeft = (1.0 * driveMotorLeft->GetSensorCollection().GetQuadraturePosition() / 4096) * 360;
    

    if (stick->GetRawButton(6) and !rotateLatch) {
        //want = want + 180;
        rotateLatch = true;
    }
    else if (!stick->GetRawButton(6) and rotateLatch) {
        rotateLatch = false;
    }
}