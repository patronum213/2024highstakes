using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern smartdrive Drivetrain;
extern motor LeftMotor1;
extern motor LeftMotor2;
extern motor LeftMotor3;
extern motor RightMotor1;
extern motor RightMotor2;
extern motor RightMotor3;
extern motor IntakeMotor;
extern motor ConveyorMotor;
extern motor ArmMotor;
extern digital_out DigitalOutA;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );