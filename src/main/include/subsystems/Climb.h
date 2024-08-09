#pragma once

#include <rev/CANSparkMax.h>

class Climb {
public:
    void SetHeight(float height) {
		elevatorPID.SetReference(height, rev::CANSparkMax::ControlType::kPosition);
		elevatorPID2.SetReference(height, rev::CANSparkMax::ControlType::kPosition);
	}

	bool GetHeightReached(float height, float tolerance = 0.05) {
		return abs(height - e_elevator.GetPosition()) < tolerance;
	}

    void init() {
		m_elevator.RestoreFactoryDefaults();
		m_elevator.SetInverted(true);
		m_elevator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
		elevatorPID.SetP(0.3/elevatorIPR);
		elevatorPID.SetI(0);
		elevatorPID.SetD(1/elevatorIPR);
		elevatorPID.SetFF(0.000156);
		elevatorPID.SetOutputRange(-1, 1);
		e_elevator.SetPositionConversionFactor(elevatorIPR);
		m_elevator.BurnFlash();

		m2_elevator.RestoreFactoryDefaults();
		m2_elevator.SetInverted(true);
		m2_elevator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
		elevatorPID2.SetP(0.3/elevatorIPR);
		elevatorPID2.SetI(0);
		elevatorPID2.SetD(1/elevatorIPR);
		elevatorPID2.SetFF(0.000156);
		elevatorPID2.SetOutputRange(-1, 1);
		e2_elevator.SetPositionConversionFactor(elevatorIPR);
		m2_elevator.BurnFlash();
    }


private:
	rev::CANSparkMax m_elevator{61, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController elevatorPID = m_elevator.GetPIDController();
	rev::SparkRelativeEncoder e_elevator = m_elevator.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::CANSparkMax m2_elevator{62, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController elevatorPID2 = m2_elevator.GetPIDController();
	rev::SparkRelativeEncoder e2_elevator = m2_elevator.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
	// elevator gearbox reduction
	const float elevatorGearboxReduction = 36;
	// inches per rotation of the motor
	const float elevatorIPR = 9/2.54/elevatorGearboxReduction;
} climb;