/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//testing commit
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, D        
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
triport ThreeWirePort = vex::triport( vex::PORT22 );
digital_out DigitalOutA = vex::digital_out(ThreeWirePort.A);


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void MoveStraight(int speed, int distance) {
  
};
int exponent(double base, int exponent) {
    double product = 1;
    for (size_t i = 0; i < exponent; i++)
    {
      product *= base; 
    }
      
};
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  IntakeMotor.setMaxTorque(50, pct);
  int leftsidepower;
  int rightsidepower;
  float FBsensitivity = 1;
  float LRsensitivity = 0.7; 
  bool goalintakeopen = false;
  bool buttonPreviouslyPressed = false;
  while (true) {
    //Driving Control
    //controller dead zone
    int deadzonepct  = 15;
    int Axis3 = 
    Controller1.Axis3.position(percent) > 0 ? (Controller1.Axis3.position(percent) - deadzonepct) * 100/deadzonepct : 
    Controller1.Axis3.position(percent) < 0 ? (Controller1.Axis3.position(percent) + deadzonepct) * 100/deadzonepct : 0;
    int Axis1 = 
    Controller1.Axis1.position(percent) > 0 ? (Controller1.Axis1.position(percent) - deadzonepct) * 100/deadzonepct : 
    Controller1.Axis1.position(percent) < 0 ? (Controller1.Axis1.position(percent) + deadzonepct) * 100/deadzonepct : 0;
    //input power curve
    double curveconstant = 1.05;
    Axis1 = (100*(exponent(curveconstant, Axis1)-1))/(exponent(curveconstant, 100)-1);   
    Axis3 = (100*(exponent(curveconstant, Axis3)-1))/(exponent(curveconstant, 100)-1);   
    //sensitivity
    Axis1 = Axis1*FBsensitivity;
    Axis3 = Axis3*LRsensitivity;
    //set motor powers
    leftsidepower = (Axis3 + Axis1);
    rightsidepower = (Axis3 - Axis1);
    LeftMotor1.spin(directionType::fwd, leftsidepower, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, leftsidepower, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, leftsidepower, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, rightsidepower, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, rightsidepower, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, rightsidepower, velocityUnits::pct);

    //intake and conveyor (toggle by L2)
    if(Controller1.ButtonL2.pressing() && goalintakeopen == true && buttonPreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      DigitalOutA.set(false);
      goalintakeopen = false;
    }
    else if(Controller1.ButtonL2.pressing() && goalintakeopen == false && buttonPreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      DigitalOutA.set(true);
      goalintakeopen = true;
    }

    if (Controller1.ButtonB.pressing()) {buttonPreviouslyPressed = true;}
    else {buttonPreviouslyPressed = false;}


    //Conveyor & intake controls
    if(Controller1.ButtonR1.pressing()) {//out
      ConveyorMotor.spin(directionType::fwd, 100, velocityUnits::pct); 
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct); 
    }
    else if(Controller1.ButtonR2.pressing()) { //in
      ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct); 
      IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct); 
    }
    else if(Controller1.ButtonR1.pressing() && Controller1.ButtonY.pressing()) {//low speed mode for testing while holding Y
      ConveyorMotor.spin(directionType::fwd, 25, velocityUnits::pct); 
      IntakeMotor.spin(directionType::fwd, 25, velocityUnits::pct); 
    }
    else if(Controller1.ButtonR2.pressing() && Controller1.ButtonY.pressing()) { 
      ConveyorMotor.spin(directionType::rev, 25, velocityUnits::pct); 
      IntakeMotor.spin(directionType::rev, 25, velocityUnits::pct); 
    }
    else {
      ConveyorMotor.spin(directionType::fwd, 0, velocityUnits::pct);
      IntakeMotor.spin(directionType::fwd, 0, velocityUnits::pct);
    }

    Brain.Screen.setCursor(1, 1);
    //Brain.Screen.print("limiter = %.2f  \n",limiter);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
