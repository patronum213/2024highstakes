#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotor1 = motor(PORT1, ratio6_1, false);
motor LeftMotor2 = motor(PORT2, ratio6_1, true);
motor LeftMotor3 = motor(PORT12, ratio6_1, true);
motor RightMotor1 = motor(PORT8, ratio6_1, true);
motor RightMotor2 = motor(PORT9, ratio6_1, false);
motor RightMotor3 = motor(PORT10, ratio6_1, false);
motor ConveyorMotor = motor(PORT20, ratio18_1, true);
motor IntakeMotor = motor(PORT19, ratio18_1, false);
motor ArmMotor = motor(PORT6, ratio18_1, false);
gyro TurnGyroSmart = gyro(Brain.ThreeWirePort.D);
//smartdrive Drivetrain= smartdrive(LeftMotor1, RightMotor1, TurnGyroSmart, 319.19, 320, 130, mm, 1);
//digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(8, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  TurnGyroSmart.startCalibration(1);
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (TurnGyroSmart.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}