#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <complex.h>
#include <frc/Timer.h>
#include <string.h>
#include "cameraserver/CameraServer.h"

#include "states.h"
#include "subsystems/Swerve.h"
#include "subsystems/IntakeShooter.h"
#include "subsystems/NoteHandler.h"
#include "subsystems/Climb.h"
#include "subsystems/Limelight.h"
using namespace std;

unsigned int x = 0; // current autoPos setpoint index
float pitch;
float turnSpeed = 0.65;
float tx;
float ty;
bool targetValid;
float angleSetpoint = 0;
float tROffset = 0;

struct autoValue
{
	complex<float> pos;
	float angle = 0;
	enum AutoState startingState = AutoState::ADRIVING;
};

frc::XboxController controller{0};
frc::Joystick key_pad{1};

frc::Timer timer;

autoValue autoPose[3] = {
	{complex<float>(0, 0), 0, AutoState::AAIMING},
	{complex<float>(60,0), 0, AutoState::AINTAKING},
	{complex<float>(25,0), 0, AutoState::AAIMING}
};


class Robot : public frc::TimedRobot{
public:
	

	void RobotInit() override;
	void RobotPeriodic() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void DisabledInit() override;
	void DisabledPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

	void SimulationInit() override;
	void SimulationPeriodic() override;

private:
	
};