#pragma once
enum TeleopState {
    DEFAULT,
    DEFENDING,
    DISTAIM,
    DISTRAMP,
    NOTEJECT,
    PODAIM,
    ARMDEFAULT,
    INTAKING,
    NOTEALIGN1,
    NOTEALIGN2,
    NOTEALIGN3,
    AIMING,
    RAMPING,
    SHOOTING,
    AMPTRANSFER,
    TRAPTRANSFER,
    INTAKECLEAR,
    TRAPCLIMBUP,
    TRAPCLIMBDOWN,
    TRAPSCOREREADY,
    WAITINGTOSCORE,
    TRAPSCORE,
    CLIMBUP,
    CLIMBDOWN,
    AMPSCORE,
    AMPOS,
    WAITFORCLIMB,

    NUM_OF_STATES // the number of states: do not remove
};

enum TeleopState teleopState = TeleopState::DEFAULT;
enum TeleopState lastTeleopState = TeleopState::DEFAULT;

// Array of function pointers for state initialization code
void (*TelInit[TeleopState::NUM_OF_STATES])();
// Array of function pointers for the state periodic code
void (*TelPeriodic[TeleopState::NUM_OF_STATES])();

void defineTeleopStateFunctions();


enum AutoState {
    ADRIVING,
    AINTAKING,
    AAIMING,
    ARAMPING,
    ASHOOTING,
    ANOTEALIGN1,
    ANOTEALIGN2,
    ANOTEALIGN3,

    ANUM_OF_STATES // the number of states: do not remove
};

enum AutoState autoState = AutoState::ADRIVING;
enum AutoState lastAutoState = AutoState::ADRIVING;

// Array of function pointers for state initialization code
void (*AutoInit[AutoState::ANUM_OF_STATES])();
// Array of function pointers for the state periodic code
void (*AutoPeriodic[AutoState::ANUM_OF_STATES])();

void defineAutoStateFunctions();