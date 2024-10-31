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
//test
#include "vex.h"
#include <cmath>
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
triport ThreeWirePort = vex::triport( vex::PORT22 );//goal hook
digital_out GoalPneumatics = vex::digital_out(ThreeWirePort.H);
digital_out LobsterPneumatics = vex::digital_out(ThreeWirePort.B);
digital_out IntakePneumatics = vex::digital_out(ThreeWirePort.C);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void resetMotorEncoders(void) {
  LeftMotor1.resetPosition(); 
  LeftMotor2.resetPosition(); 
  LeftMotor3.resetPosition();
  RightMotor1.resetPosition();
  RightMotor2.resetPosition();
  RightMotor3.resetPosition();
  ConveyorMotor.resetPosition();
};
float distributeNormally (float input) {
  return std::pow(2.71828, -std::pow(((4*input)-2), 2));
};
float distributeParabolically (float input) {
  return (-4*std::pow((input-0.5), 2) + 1);
};
void MoveStraight(float distance, int maxSpeed, bool fowards) {
  resetMotorEncoders();
  //wheels are 4 inches in diamametere, times pi means curcumernce is 12.56636 in
  //divided by 360 to get the inces per degree (0.0349065556)
  //times 2.3333333332 for the gearing
  //gives us the final multiplier of 0.0139626222 
  /*float distancedeg = (distance*0.0349065556)*2.3333333332;
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("dist in deg = %.2f  ", (distance*0.0349065556)*2.3333333332);*/
  //alright screw it it's revolution time
  //distance (in inches) divided by wheel curcumfrence mutiplied by gear raito
  //TODO: switch this to pid for greater accuracy.
  float distancerev = (distance/12.56636)*2.3333333332;
  if (fowards) {
    while (LeftMotor2.position(rev) < distancerev) {
    float distanceTraveledPct = (LeftMotor2.position(rev)/distancerev)*100.0;
    float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
    float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
    LeftMotor1.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    if (LeftMotor2.position(rev) > distancerev) {
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
    
    while (LeftMotor2.position(rev) > -distancerev) {
    float distanceTraveledPct = (LeftMotor2.position(rev)/-distancerev)*100.0;
    float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
    float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
    LeftMotor1.spin(directionType::rev, ajustedSpeed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::rev, ajustedSpeed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
    if (LeftMotor2.position(rev) < -distancerev) {
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

void MoveTurning(int degrees, int maxSpeed, bool isturningright) {
  resetMotorEncoders();
  //wheels are 4 inches in diamametere, times pi means curcumernce is 12.56636 in
  //wheel to wheel width is 14.75, time pi means one 360 degree turn is 46.34845 in covered
  //divided by 360 is 0.12875 inches covered per degree of turning
  float distanceInch = degrees*0.12875;
  //using the same inch to revolution from driveStraight
  float distanceRev = (distanceInch/12.56636)*2.3333333332;
  distanceRev *= 1.032;
  if (isturningright) {
    while (LeftMotor2.position(rev) < distanceRev) {//to turn right, left wheels must go fowards while right wheels must go backwards
    float distanceTraveledPct = (LeftMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
    float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
    LeftMotor1.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
      if (LeftMotor2.position(rev) > distanceRev) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
        RightMotor1.stop();
        RightMotor2.stop();
        RightMotor3.stop();
        }
    };
  }
  else if (!isturningright) {
    while (RightMotor2.position(rev) < distanceRev) {
    float distanceTraveledPct = (RightMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
    float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
    LeftMotor1.spin(directionType::rev, ajustedSpeed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::rev, ajustedSpeed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, ajustedSpeed, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, ajustedSpeed, velocityUnits::pct);
    if (RightMotor2.position(rev) > distanceRev) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
      RightMotor1.stop();
      RightMotor2.stop();
      RightMotor3.stop();
      }
    };
  }
  
};
void TurnWithRatio(int distance, int maxSpeed, float LeftToRightRatio, bool fowards) {
  resetMotorEncoders();
  //turn with ratio uses the ratio of left wheel power to right wheel power to determine direction
  //if LtoRratio is greater than 1, it turns right, less than one and it turns left
  //ratios should be given in fractions anyway to help keep track of turns 
  //TODO: switch this to pid for greater accuracy.
  float distancerev = (distance/12.56636)*2.3333333332;
  float leftSideMultiplier = 1;
  float rightSideMultiplier = 1;
    if (LeftToRightRatio > 1) {
      rightSideMultiplier = 1/LeftToRightRatio;
      if (fowards) {
        while (LeftMotor2.position(rev) < distancerev) {
        float distanceTraveledPct = (LeftMotor2.position(rev)/distancerev)*100.0;
        float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
        float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
        
        LeftMotor1.spin(directionType::fwd, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor2.spin(directionType::fwd, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor3.spin(directionType::fwd, ajustedSpeed*leftSideMultiplier, velocityUnits::pct);
        RightMotor1.spin(directionType::fwd, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor2.spin(directionType::fwd, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor3.spin(directionType::fwd, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        if (LeftMotor2.position(rev) > distancerev) {
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
        while (LeftMotor2.position(rev) > -distancerev) {
        float distanceTraveledPct = (LeftMotor2.position(rev)/-distancerev)*100.0;
        float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
        float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
        LeftMotor1.spin(directionType::rev, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor2.spin(directionType::rev, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor3.spin(directionType::rev, ajustedSpeed*leftSideMultiplier, velocityUnits::pct);
        RightMotor1.spin(directionType::rev, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor2.spin(directionType::rev, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor3.spin(directionType::rev, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        if (LeftMotor2.position(rev) < -distancerev) {
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
    else if (LeftToRightRatio < 1) {
      leftSideMultiplier = LeftToRightRatio;
      if (fowards) {
        while (RightMotor2.position(rev) < distancerev) {
        float distanceTraveledPct = (RightMotor2.position(rev)/distancerev)*100.0;
        float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
        float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
        
        LeftMotor1.spin(directionType::fwd, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor2.spin(directionType::fwd, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor3.spin(directionType::fwd, ajustedSpeed*leftSideMultiplier, velocityUnits::pct);
        RightMotor1.spin(directionType::fwd, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor2.spin(directionType::fwd, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor3.spin(directionType::fwd, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        if (RightMotor2.position(rev) > distancerev) {
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
        while (RightMotor2.position(rev) > -distancerev) {
        float distanceTraveledPct = (RightMotor2.position(rev)/-distancerev)*100.0;
        float distributedSpeed = distributeParabolically(distanceTraveledPct/100.0)*100.0;
        float ajustedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
        LeftMotor1.spin(directionType::rev, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor2.spin(directionType::rev, ajustedSpeed*leftSideMultiplier, velocityUnits::pct); 
        LeftMotor3.spin(directionType::rev, ajustedSpeed*leftSideMultiplier, velocityUnits::pct);
        RightMotor1.spin(directionType::rev, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor2.spin(directionType::rev, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        RightMotor3.spin(directionType::rev, ajustedSpeed*rightSideMultiplier, velocityUnits::pct);
        if (RightMotor2.position(rev) < -distancerev) {
          LeftMotor1.stop(); 
          LeftMotor2.stop(); 
          LeftMotor3.stop();
          RightMotor1.stop();
          RightMotor2.stop();
          RightMotor3.stop();
          }
        };
      }
    };
  
  
  
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
  LeftMotor1.setStopping(hold); 
  LeftMotor2.setStopping(hold); 
  LeftMotor3.setStopping(hold);
  RightMotor1.setStopping(hold);
  RightMotor2.setStopping(hold);
  RightMotor3.setStopping(hold);
  resetMotorEncoders();
  GoalPneumatics.set(true);
  ////////////////////////////////////////////////////////////////////////////////
  /////////////////////////RED NEGATIVE SIDE 4-RING AUTO//////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  //6 points, preload + negative side stack + 2 center stacks; touches ladder
  //13.5 seconds
  //slot 1
  /*MoveStraight(18, 70, false);//move out to get the goal
  GoalPneumatics.set(false);//grab it
  ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct);//start spinning the conveyor
  MoveTurning(60, 50, true);//turn towards the first stack
  MoveStraight(22, 70, true);//go in to it
  MoveStraight(18, 50, false);// reverse to not pick up the blue ring on top
  MoveTurning(23, 50, true);//turn towards the center stack(s)
  MoveStraight(23, 70, true);//drive in to center facing one
  TurnWithRatio(38, 100, 1.0/3.0, false);//back out and align with the other
  MoveTurning(62, 50, false);//turn towards it
  MoveStraight(24, 70, true);//run in to it
  MoveStraight(5, 70, false);//reverse
  MoveTurning(75, 50, true);//turn towards the ladder
  MoveStraight(40, 70, true);//bump in to it
  */
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////BLUE NEGATIVE SIDE 4-RING AUTO//////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  //6 points, preload + negative side stack + 2 center stacks; touches ladder
  //13.5 seconds
  //slot 2
  /*MoveStraight(18, 70, false);//move out to get the goal
  GoalPneumatics.set(false);//grab it
  ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct);//start spinning the conveyor
  MoveTurning(60, 50, false);//turn towards the first stack
  MoveStraight(22, 70, true);//go in to it
  MoveStraight(18, 50, false);// reverse to not pick up the blue ring on top
  MoveTurning(23, 50, false);//turn towards the center stack(s)
  MoveStraight(23, 70, true);//drive in to center facing one
  TurnWithRatio(38, 100, 3.0/1.0, false);//back out and align with the other
  MoveTurning(62, 50, true);//turn towards it
  MoveStraight(24, 70, true);//run in to it
  MoveStraight(5, 70, false);//reverse
  MoveTurning(75, 50, false);//turn towards the ladder
  MoveStraight(40, 70, true);//bump in to it
  */
  ////////////////////////////////////////////////////////////////////////////////
  /////////////////////////RED POSITIVE SIDE 3-RING AUTO//////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  //5 points, preload + middle inverted stack + positive side stack
  //11 seconds
  //slot 4
  /*MoveStraight(18, 70, false);//move to goal
  GoalPneumatics.set(false);//grab it
  ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct);
  LobsterPneumatics.set(true);//pull the rollers up REMEMBER TO CHANGE WHEN THE ACTUATOR IS INSTALLED ///
  MoveTurning(9, 50, true);//turn towards the center stack
  MoveStraight(18, 30, true);//go up to/through it
  LobsterPneumatics.set(false);//let the rollers fall
  MoveStraight(10, 30, false);//pull it back
  MoveTurning(155, 50, false);//turn towards the second stack
  MoveStraight(35, 70, true);//go to it and intake the ring
  MoveTurning(127, 70, false);//turn towards the tower
  MoveStraight(21, 70, true);//go and touch it
  */
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////BLUE POSITIVE SIDE 3-RING AUTO//////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  //5 points, preload + middle inverted stack + positive side stack
  //11 seconds
  //slot 4
  /*MoveStraight(18, 70, false);//move to goal
  GoalPneumatics.set(false);//grab it
  ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct);
  LobsterPneumatics.set(true);//pull the rollers up REMEMBER TO CHANGE WHEN THE ACTUATOR IS INSTALLED ///
  MoveTurning(9, 50, false);//turn towards the center stack
  MoveStraight(18, 30, true);//go up to/through it
  LobsterPneumatics.set(false);//let the rollers fall
  MoveStraight(10, 30, false);//pull it back
  MoveTurning(155, 50, true);//turn towards the second stack
  MoveStraight(35, 70, true);//go to it and intake the ring
  MoveTurning(127, 70, true);//turn towards the tower
  MoveStraight(21, 70, true);//go and touch it
  */
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////SKILLS///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
ConveyorMotor.spin(directionType::rev, 100, velocityUnits::pct);
wait(2, sec);
MoveStraight(12, 60, true);
MoveTurning(90, 50, false);
MoveStraight(13, 60, false);
GoalPneumatics.set(false);
MoveTurning(180, 50, false);
MoveStraight(50, 60, true);
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
  int leftsidepower;
  int rightsidepower;
  float FBsensitivity = 1.0;
  float LRsensitivity = 0.7; 
  bool goalintakeopen = false;
  bool L2PreviouslyPressed = false;
  bool L1PreviouslyPressed = false;
  bool R2PreviouslyPressed = false;
  bool intakeActive = false;
  resetMotorEncoders();
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

    //goal pneumatics (toggled by L2)
    if(Controller1.ButtonL2.pressing() && goalintakeopen == true && L2PreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      GoalPneumatics.set(false);
      goalintakeopen = false;
    }
    else if(Controller1.ButtonL2.pressing() && goalintakeopen == false && L2PreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      GoalPneumatics.set(true);
      goalintakeopen = true;
    }

    if (Controller1.ButtonL2.pressing()) {L2PreviouslyPressed = true;}
    else {L2PreviouslyPressed = false;
    }
    
    //goal pneumatics (toggled by L1)
    if(Controller1.ButtonL1.pressing()) { //If L2 is pressed while the limiter is 1
      LobsterPneumatics.set(true);
    }
    else if(!Controller1.ButtonL1.pressing()) { //If L2 is pressed while the limiter is 1
      LobsterPneumatics.set(false);
    }
    /*if(Controller1.ButtonL1.pressing() && goalintakeopen == true && L1PreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      LobsterPneumatics.set(false);
      //goalintakeopen = false;
    }
    else if(Controller1.ButtonL1.pressing() && goalintakeopen == false && L1PreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      LobsterPneumatics.set(true);
      //goalintakeopen = true;
    }*/

    if (Controller1.ButtonL1.pressing()) {L1PreviouslyPressed = true;}
    else {L1PreviouslyPressed = false;
    }

    //intake and conveyor (toggled by R2)
    if(Controller1.ButtonR2.pressing() && intakeActive == true && R2PreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      intakeActive = false;
    }
    else if(Controller1.ButtonR2.pressing() && intakeActive == false && R2PreviouslyPressed == false) { //If L2 is pressed while the limiter is 1
      intakeActive = true;
    }
    if (Controller1.ButtonR2.pressing()) {R2PreviouslyPressed = true;}
    else {R2PreviouslyPressed = false;
    }
    
    
    if(Controller1.ButtonR1.pressing() && Controller1.ButtonY.pressing()) {//out but slow
      ConveyorMotor.spin(directionType::fwd, 25, velocityUnits::pct); 
      IntakeMotor.spin(directionType::fwd, 25, velocityUnits::pct); 
    }
    else if(intakeActive && Controller1.ButtonY.pressing()) { //in but slow
      ConveyorMotor.spin(directionType::rev, 25, velocityUnits::pct); 
      IntakeMotor.spin(directionType::rev, 25, velocityUnits::pct); 
    }
    else if(Controller1.ButtonR1.pressing()) {//out
      ConveyorMotor.spin(directionType::fwd, 100, velocityUnits::pct); 
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct); 
    }
    else if(intakeActive) { //in
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