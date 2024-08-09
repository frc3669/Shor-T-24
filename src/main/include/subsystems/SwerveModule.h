#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkMax.h>
#include <complex.h>
#include <string>
#include "angleMath.h"

using namespace std;

class Module{
public:
    Module(int modID, complex<float> pos){
        turnVector = pos*complex<float>(0, 1)/abs(pos);
        dMotor = new ctre::phoenix6::hardware::TalonFX(modID+10, "CTREdevices");
        sMotor = new ctre::phoenix6::hardware::TalonFX(modID+20, "CTREdevices");
        encoder = new ctre::phoenix6::hardware::CANcoder(modID+30, "CTREdevices");
    }
    void init() {
        dMotor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        ctre::phoenix6::configs::TalonFXConfiguration configs{};
		/* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
		configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
		configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
		configs.TorqueCurrent.PeakForwardTorqueCurrent = 50;  // Peak output of 40 amps
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -50; // Peak output of 40 amps
        configs.CurrentLimits.StatorCurrentLimit = 60;
        configs.CurrentLimits.SupplyCurrentLimit = 40;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        dMotor->GetConfigurator().Apply(configs);
    }
    void set(complex<float> rVector, float turnRate){
        complex<float> modVector = getVelocity(rVector, turnRate);
        float throttle = abs(modVector);
        angle = encoder->GetAbsolutePosition().GetValue().value()*(M_PI*2);
        float error = arg(modVector)-angle;
        if (throttle < 0.001) {
            error = 0;
        }
        am::limit(error);
        if (abs(error) > (M_PI/2)){
            error += M_PI;
            am::limit(error);
            throttle *= -1;
        }
        sMotor->Set(error/M_PI);
        auto friction_torque = (throttle > 0) ? 1_A : -1_A; // To account for friction, we add this to the arbitrary feed forward
        /* Use torque velocity */
        dMotor->SetControl(m_velocity.WithVelocity(throttle*90_tps).WithFeedForward(friction_torque));
    }
    complex<float> getPositionChange() {
        float motorPos = dMotor->GetPosition().GetValue().value();
        motorPosChg = motorPos - motorPosOld;
        complex<float> modPosChange = polar<float>(motorPosChg*3.9*M_PI/6.75, angle);
        motorPosOld = motorPos;
        return modPosChange;
    }

    void resetEncoders() {
        motorPosOld = 0;
        dMotor->SetPosition(0_tr);
    }

    float getMotorPos() {
        return dMotor->GetPosition().GetValue().value();
    }

    float getMotorPosChange() {
        return motorPosChg;
    }

    complex<float> getVelocity(complex<float> rVector, float turnRate){
        return rVector+turnVector*turnRate;
    }

private:
    ctre::phoenix6::hardware::TalonFX *dMotor;
    ctre::phoenix6::hardware::CANcoder *encoder;
	ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocity{0_tps, 0_tr_per_s_sq, 0_A, 1, false};
    ctre::phoenix6::hardware::TalonFX *sMotor;
    complex<float> turnVector;
    complex<float> modPosChange;
    float motorPosChg = 0;
    float motorPosOld = 0;
    float angle;
};
/*

# # # # # # # # # # # # # # # # # # #
# DM:11                       DM:14 #
# SM:21                       SM:24 #
# E:31                         E:34 #
#                                   #
#                ^                  #
#                | x                #
#          y <-- +                  #
#                                   #
#                                   #
#                                   #
# DM:12                       DM:13 #
# SM:22                       SM:23 #
# E:32                         E:33 #
# # # # # # # # # # # # # # # # # # #

*/
