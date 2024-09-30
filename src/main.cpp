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
void MoveStraight(int speed, float distance, bool fowards) {
  LeftMotor2.resetPosition();
  RightMotor2.resetPosition();
  float distancedeg = (distance/0.0349065556)*0.333333;//4*pi/360 and weird gearing constant
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("dist in deg = %.2f  ", (distance/0.0349065556)*0.4285714286);
  if (fowards) {
    while (!(LeftMotor2.position(deg) > distancedeg)) {
    LeftMotor1.spin(directionType::fwd, speed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, speed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    if (LeftMotor2.position(deg) > distancedeg) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
      RightMotor1.stop();
      RightMotor2.stop();
      RightMotor3.stop();
      }
    };
  }
  else if (!fowards) {
    while (!(LeftMotor2.position(deg) < -distancedeg)) {
    LeftMotor1.spin(directionType::rev, speed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::rev, speed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, speed, velocityUnits::pct);
    if (LeftMotor2.position(deg) < -distancedeg) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
      RightMotor1.stop();
      RightMotor2.stop();
      RightMotor3.stop();
      }
    };
  }
  
}
/*
void MoveTurning(int speed, int degrees, bool isturningright) {
  LeftMotor2.resetPosition();
  RightMotor2.resetPosition();
  float degreesinwheeldeg = distance*4523.76;//4*pi*360
  
  if (distancedeg > 0) {
    while (!(LeftMotor2.position(deg) > distance)) {
    LeftMotor1.spin(directionType::fwd, speed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, speed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, speed, velocityUnits::pct);
    if (LeftMotor2.position(deg) > distance) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
      RightMotor1.stop();
      RightMotor2.stop();
      RightMotor3.stop();
      }
    };
  }
  else if (distancedeg > 0) {
    while (!(LeftMotor2.position(deg) < distance)) {
    LeftMotor1.spin(directionType::rev, speed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::rev, speed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, speed, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, speed, velocityUnits::pct);
    if (LeftMotor2.position(deg) < distance) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
      RightMotor1.stop();
      RightMotor2.stop();
      RightMotor3.stop();
      }
    };
  }
  
}*/
double exponent(double base, double exponent) {
    double product = 1;
    for (size_t i = 0; i < exponent; i++)
    {
      product *= base; 
    }
    return product;     
}
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
  LeftMotor1.setStopping(brake); 
  LeftMotor2.setStopping(brake); 
  LeftMotor3.setStopping(brake);
  RightMotor1.setStopping(brake);
  RightMotor2.setStopping(brake);
  RightMotor3.setStopping(brake);
  DigitalOutA.set(true);
  MoveStraight(40, 24, false); 
  wait(200, msec);
  DigitalOutA.set(false);
  wait(200, msec);
  ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct); 
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct); 
  wait(1000, msec);
  ConveyorMotor.stop();
  IntakeMotor.stop();
  MoveStraight(40, 20, false); 
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
  LeftMotor1.setStopping(coast); 
  LeftMotor2.setStopping(coast); 
  LeftMotor3.setStopping(coast);
  RightMotor1.setStopping(coast);
  RightMotor2.setStopping(coast);
  RightMotor3.setStopping(coast);
  // User control code here, inside the loop
  IntakeMotor.setMaxTorque(50, pct);
  int leftsidepower;
  int rightsidepower;
  float FBsensitivity = 1.0;
  float LRsensitivity = 0.7; 
  bool goalintakeopen = false;
  bool buttonPreviouslyPressed = false;
  LeftMotor2.resetPosition();
  while (true) {
    //Driving Control
    //controller dead zone
    int deadzonepct  = 15;
    float Axis3 = Controller1.Axis3.position(percent) > deadzonepct ? ((Controller1.Axis3.position(percent) - deadzonepct)*1.00/(100-deadzonepct))*100 : 
    Axis3 = Controller1.Axis3.position(percent) < -deadzonepct ? ((Controller1.Axis3.position(percent) + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
    float Axis1 = Controller1.Axis1.position(percent) > deadzonepct ? ((Controller1.Axis1.position(percent) - deadzonepct)*1.00/(100-deadzonepct))*100 : 
    Axis1 = Controller1.Axis1.position(percent) < -deadzonepct ? ((Controller1.Axis1.position(percent) + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
    //input power curve
    /*double curveconstant = 1.05;
    Axis1 = (100*(exponent(curveconstant, Axis1)-1))/(exponent(curveconstant, 100)-1);   
    Axis3 = (100*(exponent(curveconstant, Axis3)-1))/(exponent(curveconstant, 100)-1);   
    */

    //sensitivity
    Axis1 *= LRsensitivity;
    Axis3 *= FBsensitivity;
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

    if (Controller1.ButtonL2.pressing()) {buttonPreviouslyPressed = true;}
    else {buttonPreviouslyPressed = false;}


    //Conveyor & intake controls
    if(Controller1.ButtonR1.pressing() && Controller1.ButtonY.pressing()) {//low speed mode for testing while holding Y
      ConveyorMotor.spin(directionType::fwd, 25, velocityUnits::pct); 
      IntakeMotor.spin(directionType::fwd, 25, velocityUnits::pct); 
    }
    else if(Controller1.ButtonR2.pressing() && Controller1.ButtonY.pressing()) { 
      ConveyorMotor.spin(directionType::rev, 25, velocityUnits::pct); 
      IntakeMotor.spin(directionType::rev, 25, velocityUnits::pct); 
    }
    else if(Controller1.ButtonR1.pressing()) {//out
      ConveyorMotor.spin(directionType::fwd, 100, velocityUnits::pct); 
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct); 
    }
    else if(Controller1.ButtonR2.pressing()) { //in
      ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct); 
      IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct); 
    }
    else {
      ConveyorMotor.spin(directionType::fwd, 0, velocityUnits::pct);
      IntakeMotor.spin(directionType::fwd, 0, velocityUnits::pct);
    }

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Controller Axis3 pct = %d  ", Controller1.Axis3.position(percent));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Controller Axis1 pct = %d  ", Controller1.Axis1.position(percent));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Var Axis1 = %d  ", Axis1);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Var Axis3 = %d  ", Axis3);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("lm2posdeg = %.2f    ", LeftMotor2.position(deg));//
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
