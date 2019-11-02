#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
vex::brain Brain;
vex::controller Controller1 = vex::controller();


vex::motor rightDrive = vex::motor(vex::PORT10,vex::gearSetting::ratio18_1,true);
vex::motor rightDrive2 = vex::motor(vex::PORT9,vex::gearSetting::ratio18_1,true);

vex::motor leftDrive = vex::motor(vex::PORT8,vex::gearSetting::ratio18_1,false);
vex::motor leftDrive2 = vex::motor(vex::PORT7,vex::gearSetting::ratio18_1,false);

vex::motor intake = vex::motor(vex::PORT2,vex::gearSetting::ratio6_1,true);
vex::motor catapult = vex::motor(vex::PORT1,vex::gearSetting::ratio18_1,false);
vex::motor capBaller = vex::motor(vex::PORT21,vex::gearSetting::ratio18_1,false);
vex::motor catapult2 = vex::motor(vex::PORT4,vex::gearSetting::ratio18_1,true);

//vex::gyro Gyro = vex::gyro(Brain.ThreeWirePort.B); //D
vex::pot armPot = vex::pot(Brain.ThreeWirePort.G);
vex::pot catapultPot = vex::pot(Brain.ThreeWirePort.B);
vex::encoder leftEncoder = vex::encoder(Brain.ThreeWirePort.E);
vex::encoder rightEncoder = vex::encoder(Brain.ThreeWirePort.C);
