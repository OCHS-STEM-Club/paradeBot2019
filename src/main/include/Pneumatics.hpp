#pragma once

#include <iostream>
#include <frc/WPILib.h>

class PneumaticManager {
    private:
    frc::Compressor *compressor;
    frc::Solenoid *solonoid;

    frc::Joystick *stick;

    bool rotateLatch;

    public:
    PneumaticManager();
    void air();
};