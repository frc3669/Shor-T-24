// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <angleMath.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>

class IntakeShooter {
public:
	frc::DigitalInput eye0{0};
	frc::DigitalInput eye1{1};
    frc::DigitalInput eye2{2};
 	void SetAngle(float angle) {
		this->angleSetpoint = angle;
 	}
	void SetP(float P = 0.0125) {
		this->P = P;
	}
	void SetI(float I = 0) {
		this->I = I;
	}
	void SetD(float D = 0) {
		this->D = D;
	}
	void SetF(float F = 0) {
		this->F = F;
	}
	void SetOutputRange(float min = -0.25, float max = 0.3) {
		this->max = max;
		this->min = min;
	}
 	void RunAnglePID() {
		float currentAngle = GetAngle();
		angleError = angleSetpoint - currentAngle;
		am::limitDeg(angleError);
		float output = angleError*P;
		accumulator += angleError*I;
		output += accumulator;
		float angleChange = currentAngle - lastAngle;
		am::limitDeg(angleChange);
		output -= angleChange*D;
		output += F;
		if (output > max) {
			output = max;
		}
		if (output < min) {
			output = min;
		}
		m_angle.Set(output);
 	}
	bool GetAngleReached(float tolerance = 2) {
		return abs(angleError) < tolerance;
	}
	void SetIntakeSpeed(float inPerSec) {
		intakePID.SetReference(inPerSec*60, rev::CANSparkMax::ControlType::kVelocity);
	}
    void SetIntake(float percent){
        m_intake.Set(percent/100);
    }
	void SetShooterSpeed(float inPerSec) { 
		/* Use torque velocity */
		m1_shooter.SetControl(s_velocity.WithVelocity(inPerSec/shooterIPR*1_tps).WithFeedForward(20_A));
		m2_shooter.SetControl(s_velocity.WithVelocity(inPerSec/shooterIPR*1_tps).WithFeedForward(15_A));
	}
	void SetShooter(float speed) {
		m1_shooter.Set(speed/100);
		m2_shooter.Set(speed/100);
	}
	int GetNotePresent() {
		return eye0.Get() || eye1.Get() || eye2.Get();
	}
	double GetAngle() {
		return -e_abs_angle.GetDistance();
	}
 	void init() {
		m_intake.RestoreFactoryDefaults();
        m_intake.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
		m_intake.SetInverted(true);
		intakePID.SetP(6e-5);
		intakePID.SetI(0);
		intakePID.SetD(0);
		intakePID.SetFF(0.000015);
		e_intake.SetPositionConversionFactor(intakeIPR);
		m_intake.BurnFlash();
		
 		m_angle.RestoreFactoryDefaults();
        m_angle.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
 		m_angle.SetInverted(false);
 		// anglePID.SetP(0.1/angleDPR);
 		// anglePID.SetI(0);
 		// anglePID.SetD(6/angleDPR);
 		// anglePID.SetFF(-0.05);
   //      anglePID.SetOutputRange(-0.25, 0.4);
 	// 	e_angle.SetPositionConversionFactor(angleDPR);
		e_abs_angle.SetDistancePerRotation(360.0);
		e_abs_angle.SetPositionOffset(351);
 		m_angle.BurnFlash();
		lastAngle = GetAngle();
        // m1_shooter.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        // m2_shooter.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        // ctre::phoenix6::configs::TalonFXConfiguration configs{};
        // configs.CurrentLimits.StatorCurrentLimit = 150;
		// /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		// configs.Slot1.kP = 7; // An error of 1 rotation per second results in 5 amps output
		// configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
		// configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

		// configs.TorqueCurrent.PeakForwardTorqueCurrent = 100;  // Peak output of 40 amps
		// configs.TorqueCurrent.PeakReverseTorqueCurrent = -100; // Peak output of 40 amps

        // m1_shooter.GetConfigurator().Apply(configs);
        // m2_shooter.GetConfigurator().Apply(configs);
 	}
	
private:
	float angleSetpoint = 15;
	float angleError;
	float lastAngle;
	float accumulator = 0;
	float P = 0.008;
	float I = 0;
	float D = 0;
	float F = -0.015;
	float max = 0.25;
	float min = -0.25;
	rev::CANSparkMax m_intake{42, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController intakePID = m_intake.GetPIDController();
	rev::SparkRelativeEncoder e_intake = m_intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	ctre::phoenix6::controls::VelocityTorqueCurrentFOC s_velocity{0_tps, 0_tr_per_s_sq, 0_A, 1, false};
	ctre::phoenix6::hardware::TalonFX m1_shooter{43, "CTREdevices"};
	ctre::phoenix6::hardware::TalonFX m2_shooter{44, "CTREdevices"};

 	rev::CANSparkMax m_angle{41, rev::CANSparkMax::MotorType::kBrushless};
 	// rev::SparkPIDController anglePID = m_angle.GetPIDController();
 	// rev::SparkRelativeEncoder e_angle = m_angle.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	frc::DutyCycleEncoder e_abs_angle{9};
	
	const float intakeGearboxReduction = 9;
	// inches per rotation of the intake motor
	const float intakeIPR = M_PI*2/intakeGearboxReduction;
	// inches per rotation of the shooter motors
	const float shooterIPR = M_PI*4;

const float angleGearboxReduction = 60;
// degrees per rotation of the angle motor
const float angleDPR = 360/angleGearboxReduction;
} intakeShooter;
