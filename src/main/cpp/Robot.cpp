// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//test

#include "Robot.h"
#include "math.h"

using namespace std;

void Robot::RobotInit()
{
	intakeShooter.init();
	swerve.init();
	arm.init();
	climb.init();
	defineAutoStateFunctions();
	// Creates UsbCamera and MjpegServer [1] and connects them
	//frc::CameraServer::StartAutomaticCapture();
	//frc::CameraServer::PutVideo("Blur", 640, 480);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	swerve.SetPosition(autoPose[0].pos);
	autoState = autoPose[0].startingState;
	lastAutoState = autoPose[0].startingState;
}
void Robot::AutonomousPeriodic() {
	tx = ll.getSpeakerYaw();
	ty = ll.getSpeakerPitch();
	if (autoState != AutoState::AINTAKING) {
		pitch = 0.0038*pow(ty, 2)+0.6508*ty+64.3899;
		intakeShooter.SetAngle(pitch);
	}
	intakeShooter.RunAnglePID();
	swerve.RunPID(tx);
	targetValid = ll.getTargetValid();
	AutoPeriodic[autoState]();
	if (autoState != lastAutoState) {
		AutoInit[autoState]();
	}
	frc::SmartDashboard::PutNumber("angle", intakeShooter.GetAngle());
	frc::SmartDashboard::PutNumber("robot state", teleopState);
	frc::SmartDashboard::PutBoolean("note detected", intakeShooter.eye2.Get());
	frc::SmartDashboard::PutNumber("posx", swerve.pos.real());
	frc::SmartDashboard::PutNumber("posy", swerve.pos.imag());
	frc::SmartDashboard::PutNumber("posErrorx", swerve.posError.real());
	frc::SmartDashboard::PutNumber("posErrory", swerve.posError.imag());
	frc::SmartDashboard::PutNumber("posChangex", swerve.posChange.real());
	frc::SmartDashboard::PutNumber("posChangy", swerve.posChange.imag());
	frc::SmartDashboard::PutNumber("mod0Changex", swerve.GetModulePosChange(0).real());
	frc::SmartDashboard::PutNumber("mod0Changey", swerve.GetModulePosChange(0).imag());
	frc::SmartDashboard::PutNumber("mod1Changex", swerve.GetModulePosChange(1).real());
	frc::SmartDashboard::PutNumber("mod1Changey", swerve.GetModulePosChange(1).imag());
	frc::SmartDashboard::PutNumber("mod2Changex", swerve.GetModulePosChange(2).real());
	frc::SmartDashboard::PutNumber("mod2Changey", swerve.GetModulePosChange(2).imag());
	frc::SmartDashboard::PutNumber("mod3Changex", swerve.GetModulePosChange(3).real());
	frc::SmartDashboard::PutNumber("mod3Changey", swerve.GetModulePosChange(3).imag());
	frc::SmartDashboard::PutNumber("motor0Change", swerve.GetMotorPosChange(0));
	frc::SmartDashboard::PutNumber("motor1Change", swerve.GetMotorPosChange(1));
	frc::SmartDashboard::PutNumber("motor2Change", swerve.GetMotorPosChange(2));
	frc::SmartDashboard::PutNumber("motor3Change", swerve.GetMotorPosChange(3));
	frc::SmartDashboard::PutNumber("motor0Pos", swerve.GetMotorPos(0));
	frc::SmartDashboard::PutNumber("motor1Pos", swerve.GetMotorPos(1));
	frc::SmartDashboard::PutNumber("motor2Pos", swerve.GetMotorPos(2));
	frc::SmartDashboard::PutNumber("motor3Pos", swerve.GetMotorPos(3));

	lastAutoState = autoState;
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){
	tROffset = 0;
	lastTeleopState = teleopState;
	frc::SmartDashboard::PutNumber("Pitch: ", pitch);
	// this switch case runs for each state
	// 56.814 podium angle
	switch (teleopState) {
		case TeleopState::AIMING:
			turnSpeed = 0.3;
			if (key_pad.GetRawButton(12) && intakeShooter.GetNotePresent()) {
				timer.Restart();
				teleopState = TeleopState::RAMPING;
			}
			if (key_pad.GetRawButton(9) && intakeShooter.GetNotePresent()) {
				timer.Restart();
				teleopState = TeleopState::DISTRAMP;
			}
			if (!key_pad.GetRawButton(11)) {
				teleopState = TeleopState::DEFAULT;
			}
			tROffset = -ll.getSpeakerYaw() / 117;
			ty = ll.getSpeakerPitch();
			pitch = 0.0038*pow(ty, 2)+0.6508*ty+64.3899;
			intakeShooter.SetAngle(pitch);
			break;
		case TeleopState::DISTAIM:
			if (key_pad.GetRawButton(12) && intakeShooter.GetNotePresent()) {
				timer.Restart();
				teleopState = TeleopState::DISTRAMP;
			}
			if (!key_pad.GetRawButton(9)) {
				teleopState = TeleopState::DEFAULT;
			}
			break;
		case TeleopState::PODAIM:
			if (key_pad.GetRawButton(12) && intakeShooter.GetNotePresent()) {
				timer.Restart();
				teleopState = TeleopState::RAMPING;
			}
			if (!key_pad.GetRawButton(13)) {
				teleopState = TeleopState::DEFAULT;
			}
			break;
		case TeleopState::RAMPING:
			if (timer.HasElapsed(1.1_s)){
				teleopState = TeleopState::SHOOTING;
			}
			break;
		case TeleopState::DISTRAMP:
			if (timer.HasElapsed(1.1_s)){
				teleopState = TeleopState::SHOOTING;
			}
			break;
		case TeleopState::SHOOTING:
			if (timer.HasElapsed(1.3_s)){
				teleopState = TeleopState::DEFAULT;
			}
			break;
		// Amp procedure
		case TeleopState::AMPTRANSFER:
			if (!intakeShooter.GetNotePresent()) {
				teleopState = TeleopState::AMPOS;
			}
			if (key_pad.GetRawButtonPressed(14) && key_pad.GetRawButton(4) && key_pad.GetRawButton(7)) {
				teleopState = TeleopState::NOTEJECT;
			}
			break;
		case TeleopState::AMPOS:
			if (key_pad.GetRawButtonPressed(2)) {
				teleopState = TeleopState::AMPSCORE;
			}
			break;
		case TeleopState::AMPSCORE:
			if (timer.HasElapsed(0.5_s)) {
				teleopState = TeleopState::ARMDEFAULT;
			}
			break;
		case TeleopState::DEFENDING:
			if (key_pad.GetRawButtonPressed(1)) {
				teleopState = TeleopState::ARMDEFAULT;
			}
			break;
		case TeleopState::ARMDEFAULT:
			if (timer.HasElapsed(0.7_s)) {
				teleopState = TeleopState::DEFAULT;
			}
			break;
		// climb procedure
		case TeleopState::TRAPTRANSFER:
			if (!intakeShooter.GetNotePresent()){
				teleopState = TeleopState::INTAKECLEAR;
			}
			if (key_pad.GetRawButtonPressed(14) && key_pad.GetRawButton(4) && key_pad.GetRawButton(7)) {
				teleopState = TeleopState::NOTEJECT;
			}
			break;
		case TeleopState::INTAKECLEAR:
			if (intakeShooter.GetAngle() > 50) {
				teleopState = TeleopState::TRAPCLIMBUP;
			}
			break;
		case TeleopState::TRAPCLIMBUP:
			if (key_pad.GetRawButtonPressed(3)){
				teleopState = TeleopState::TRAPCLIMBDOWN;
			}
			break;
		case TeleopState::TRAPCLIMBDOWN:
			if (climb.GetHeightReached(0)) {
				teleopState = TeleopState::WAITFORCLIMB;
			}
			break;
		case TeleopState::WAITFORCLIMB:
			if (timer.HasElapsed(1_s)) {
				teleopState = TeleopState::TRAPSCOREREADY;
			}
			break;
		case TeleopState::TRAPSCOREREADY:
			if (timer.HasElapsed(0.4_s)) {
				teleopState = TeleopState::WAITINGTOSCORE;
			}
			break;
		case TeleopState::WAITINGTOSCORE:
			if (timer.HasElapsed(1.5_s)) {
				teleopState = TeleopState::TRAPSCORE;
			}
			break;
		case TeleopState::TRAPSCORE:
			if (timer.HasElapsed(1.5_s)) {
				arm.SetRollerSpeed(0);
			}
			break;
		case TeleopState::INTAKING:
			if (key_pad.GetRawButtonPressed(10)) {
				teleopState = TeleopState::DEFAULT;
			} else if (intakeShooter.eye0.Get()) {
				teleopState = TeleopState::NOTEALIGN1;
			}
			break;
		case TeleopState::NOTEALIGN1:
			if (intakeShooter.eye2.Get()) {
				intakeShooter.SetIntakeSpeed(-50);
				intakeShooter.SetShooter(-10);
			} else {
				teleopState = TeleopState::NOTEALIGN2;
			}
			break;
		case TeleopState::NOTEALIGN2:
			if (!intakeShooter.eye2.Get()) {
				intakeShooter.SetIntakeSpeed(50);
				intakeShooter.SetShooter(0);
			} else {
				teleopState = TeleopState::NOTEALIGN3;
			}
			break;
		case TeleopState::NOTEALIGN3:
			if (intakeShooter.GetNotePresent()) {
				teleopState = TeleopState::DEFAULT;
			}
			break;
		case TeleopState::NOTEJECT:
			if (timer.HasElapsed(0.65_s)) {
				teleopState = TeleopState::DEFAULT;
			}
		case TeleopState::DEFAULT:
			turnSpeed = 0.65;
			if (key_pad.GetRawButton(9)) {
				teleopState = TeleopState::DISTAIM;
			}
			if (key_pad.GetRawButtonPressed(10) && !intakeShooter.GetNotePresent()) {
				teleopState = TeleopState::INTAKING;
			}
			if (key_pad.GetRawButton(11)) {
				teleopState = TeleopState::AIMING;
			}
			if (key_pad.GetRawButton(13)) {
				teleopState = TeleopState::PODAIM;
			}
			if (key_pad.GetRawButtonPressed(14)) {
				teleopState = TeleopState::NOTEJECT;
			}
			if (key_pad.GetRawButtonPressed(1)) {
				teleopState = TeleopState::DEFENDING;
			}
			if (key_pad.GetRawButtonPressed(2)) {
				teleopState = TeleopState::AMPTRANSFER;
			}
			if (key_pad.GetRawButtonPressed(3) && key_pad.GetRawButton(4) && key_pad.GetRawButton(7)) {
				teleopState = TeleopState::TRAPTRANSFER;
			}
			break;
	}

	// this switch case only runs when the robot state changes
	if (teleopState != lastTeleopState) {
		switch (teleopState) {
			case TeleopState::DISTAIM:
				intakeShooter.SetAngle(60);
				break;
			case TeleopState::PODAIM://55.814
				intakeShooter.SetAngle(55.814);
				break;
			case TeleopState::RAMPING:
				intakeShooter.SetShooter(60);
				break;
			case TeleopState::DISTRAMP:
				intakeShooter.SetShooter(55);
				break;
			case TeleopState::SHOOTING:
				intakeShooter.SetIntake(100);
				break;
			case TeleopState::INTAKING:
				intakeShooter.SetAngle(95);
				intakeShooter.SetIntake(80);
				break;
			case TeleopState::NOTEALIGN1:
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntakeSpeed(50);
				break;
			// amp procedure:
			case TeleopState::AMPTRANSFER:
				arm.SetAngle(20);
				arm.SetHeight(0);
				arm.SetRollerSpeed(130);
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntakeSpeed(100);
				intakeShooter.SetShooter(10);
				break;
			case TeleopState::AMPOS:
				intakeShooter.SetAngle(40);
				intakeShooter.SetShooter(0);
				intakeShooter.SetIntake(0);
				arm.SetRollerSpeed(0);
				arm.SetHeight(15);
				arm.SetAngle(225);
				break;
			case TeleopState::AMPSCORE:
				timer.Restart();
				arm.SetRollerSpeed(-100);
				break;
			case TeleopState::DEFENDING:
				arm.SetHeight(20);
				arm.SetAngle(180);
				break;
			case TeleopState::ARMDEFAULT:
				timer.Restart();
				intakeShooter.SetAngle(40);
				arm.SetHeight(0);
				arm.SetAngle(0);
				break;
			// trap procedure:
			case TeleopState::TRAPTRANSFER:
				arm.SetAngle(21);
				arm.SetHeight(0);
				arm.SetRollerSpeed(130);
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntakeSpeed(100);
				intakeShooter.SetShooter(10);
				break;
			case TeleopState::INTAKECLEAR:
				arm.SetAngle(90);
				arm.SetHeight(0);
				arm.SetRollerSpeed(0);
				intakeShooter.SetAngle(60);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				break;
			case TeleopState::TRAPCLIMBUP:
				intakeShooter.SetAngle(60);
				arm.SetAngle(90);
				arm.SetHeight(0);
				arm.SetRollerSpeed(0);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(20);
				break;
			case TeleopState::TRAPCLIMBDOWN:
				intakeShooter.SetAngle(60);
				arm.SetAngle(90);
				arm.SetHeight(20.5);
				arm.SetRollerSpeed(0);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(0);
				break;
			case TeleopState::WAITFORCLIMB:
				timer.Restart();
				break;
			case TeleopState::TRAPSCOREREADY:
				timer.Restart();
				intakeShooter.SetAngle(15);
				arm.SetAngle(220);
				arm.SetHeight(20.5);
				arm.SetRollerSpeed(45);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(0);
				break;
			case TeleopState::WAITINGTOSCORE:
				timer.Restart();
				arm.SetRollerSpeed(0);
				break;
			case TeleopState::TRAPSCORE:
				timer.Restart();
				intakeShooter.SetAngle(15);
				arm.SetAngle(220);
				arm.SetHeight(20.5);
				arm.SetRollerSpeed(-100);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(0);
				break;
			case TeleopState::DEFAULT:
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				arm.SetAngle(0);
				arm.SetHeight(0);
				arm.SetRollerSpeed(0);
				break;
			case TeleopState::NOTEJECT:
				timer.Restart();
				intakeShooter.SetIntakeSpeed(-150);
				break;
		}
	}
	float dB = 0.03;
	complex<float> v = complex<float>(-controller.GetLeftY(), -controller.GetLeftX());
	complex<float> rightV = complex<float>(-controller.GetRightY(), -controller.GetRightX());
	if (abs(rightV) > 0.5) {
		angleSetpoint = arg(rightV);
	}
	float tR = 0;
	if (controller.GetLeftBumper()) {
		tROffset = swerve.GetTurnRateOffset(angleSetpoint);
	} else {
		tR = -controller.GetRightX();
	}
	float rateMultiplier = 0.3;//(controller.GetRightTriggerAxis()*0.5)+0.5;
	// apply smooth deadband
	complex<float> velocity = (abs(v)>dB) ? v*(1 - dB/abs(v))/(1-dB) : 0;
	float turnRate = (abs(tR)>dB) ? tR*(1 - dB/abs(tR))/(1-dB) : 0;
	// limit rates
	velocity *= rateMultiplier;
	turnRate *= rateMultiplier;
	turnRate += tROffset;
	swerve.set(velocity, turnRate);
	swerve.set(velocity, turnRate);
	intakeShooter.RunAnglePID();
	if (controller.GetAButtonPressed()) {
		swerve.resetPos();
	}
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
void defineTeleopStateFunctions() {
	TelInit[DEFAULT] = []() {
		intakeShooter.SetAngle(15);
		intakeShooter.SetIntakeSpeed(0);
		intakeShooter.SetShooter(0);
		arm.SetAngle(0);
		arm.SetHeight(0);
		arm.SetRollerSpeed(0);
	};
}

void defineAutoStateFunctions() {
	AutoInit[ADRIVING] = []() {
		intakeShooter.SetAngle(15);
		intakeShooter.SetIntake(0);
		intakeShooter.SetShooter(0);
		arm.SetAngle(0);
		arm.SetHeight(0);
		arm.SetRollerSpeed(0);
	};
	AutoPeriodic[ADRIVING] = []() {
		if (swerve.GetPositionReached() && x < size(autoPose)-1) {
			x++;
			swerve.SetPosition(autoPose[x].pos);
			autoState = autoPose[x].startingState;
		}
	};

	AutoInit[AINTAKING] = []() {
		intakeShooter.SetAngle(95);
		intakeShooter.SetIntake(70);
	};
	AutoPeriodic[AINTAKING] = []() {
		if (intakeShooter.eye0.Get()) {
			autoState = AutoState::ANOTEALIGN1;
		}
	};

	AutoInit[AAIMING] = []() {};
	AutoPeriodic[AAIMING] = []() {
		if (swerve.GetPositionReached() && intakeShooter.GetAngleReached(3) && abs(tx) < 5 && targetValid) {
			autoState = AutoState::ARAMPING;
		}
	};

	AutoInit[ARAMPING] = []() {
		timer.Restart();
		intakeShooter.SetShooter(55);
	};
	AutoPeriodic[ARAMPING] = []() {
		if (timer.HasElapsed(1_s)){
			autoState = AutoState::ASHOOTING;
		}
	};

	AutoInit[ASHOOTING] = []() {
		intakeShooter.SetIntake(100);
	};

	AutoPeriodic[ASHOOTING] = []() {
		if (timer.HasElapsed(1.3_s)){
			autoState = AutoState::ADRIVING;
		}
	};

	AutoInit[AutoState::ANOTEALIGN1] = []() {};
	AutoPeriodic[AutoState::ANOTEALIGN1] = []() {
		if (intakeShooter.eye2.Get()) {
			intakeShooter.SetIntakeSpeed(-50);
			intakeShooter.SetShooter(-10);
		} else {
			autoState = AutoState::ANOTEALIGN2;
		}
	};

	AutoInit[AutoState::ANOTEALIGN2] = []() {};
	AutoPeriodic[AutoState::ANOTEALIGN2] = []() {
		if (!intakeShooter.eye2.Get()) {
			intakeShooter.SetIntakeSpeed(50);
			intakeShooter.SetShooter(0);
		} else {
			autoState = AutoState::ANOTEALIGN3;
		}
	};

	AutoInit[ANOTEALIGN3] = []() {};
	AutoPeriodic[ANOTEALIGN3] = []() {
		if (intakeShooter.GetNotePresent()) {
			autoState = AutoState::ADRIVING;
		}
	};
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
