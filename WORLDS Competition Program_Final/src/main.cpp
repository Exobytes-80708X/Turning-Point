#include "robot-config.h"
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                   */
/*                                                                           */
/*---------------------------------------------------------------------------*/

vex::competition Competition;

/*---------------------------------------------------------------------------*/
/*                         Global Variable Declariations                     */
/*---------------------------------------------------------------------------*/

typedef enum {
  fwd,
  rev,
  left,
  right,
  up,
  down,
  middle,
  stop,
  on,
  off
} action;



//================ Odometry Variables ================

const double WHEEL_DIAMETER = 4.5;
const double ENCODER_WIDTH = 7.425;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*M_PI;
const int DELAY = 5;

double robotTheta = 0.0;
double robotX = 0.0;
double robotY = 0.0;

double dTheta = 0.0;
double dX = 0.0;
double dY = 0.0;

double currentLeft = 0.0;
double currentRight = 0.0;

double prevLeft = 0.0;
double prevRight = 0.0;

double dLeftVal = 0.0;
double dRightVal = 0.0;

//==================================================

int leftSpeed;
int rightSpeed;
int currentSpeedRight;
int currentSpeedLeft;
int leftEncoderValue;
int rightEncoderValue;
int potValue;

bool intakeOn = true;
bool intakeUp = false;
bool intakeDown = false;

bool autonStarted = false;
bool gyroCalibrated = false;

//GUI stuff=======

    int autonNum = 2;
    
    int buttonHeight = 80;
    int buttonWidth = 80;
    
    int rfx = 150;
    int rfy = 30;
    
    int bfx = 250;
    int bfy = 30;
    
    int rbx = 150;
    int rby = 130;
    
    int bbx = 250;
    int bby = 130;

    int sx = 350;
    int sy = 80;

    int tx = 50;
    int ty = 80;

    int dx = 370;
    int dy = 170;

//slew functions ====================================

double currentSpeed = 0.0;

double slew(double targetSpeed, double increment)
{
    if(currentSpeed < targetSpeed)
    {
        currentSpeed += increment;
    }
    if(currentSpeed >= targetSpeed)
    {
        currentSpeed = targetSpeed;
    }
    
    return currentSpeed;
}

int slewRight(int targetSpeed) {
  if (currentSpeedRight < targetSpeed) {
    currentSpeedRight += 10;
  } else if (currentSpeedRight > targetSpeed) {
    currentSpeedRight -= 10;
  }
  if(abs(currentSpeedRight) >= abs(targetSpeed))
      currentSpeedRight = targetSpeed;
    
  return currentSpeedRight;
}

int slewLeft(int targetSpeed) {
  if (currentSpeedLeft < targetSpeed) {
    currentSpeedLeft += 10;
  } else if (currentSpeedLeft > targetSpeed) {
    currentSpeedLeft -= 10;
  }
  if(abs(currentSpeedLeft) >= abs(targetSpeed))
      currentSpeedLeft = targetSpeed;
  
  return currentSpeedLeft;
}

double inchesTraveled(double encoderTicks)
{
    return (encoderTicks / 360)*WHEEL_CIRCUMFERENCE;
}

double radiansToDegrees(double radians)
{
    return radians*180.0/M_PI;
}

double degreesToRadians(double degrees)
{
    return degrees*M_PI/180.0;
}

double calcDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((y1 - y2), 2) + pow((x1 - x2), 2));
}

double calcAngleError(double targetX, double targetY)
{
    double radius = calcDistance(robotX, robotY, targetX, targetY);
    double predictedX = radius*sin(robotTheta) + robotX;
    double predictedY = radius*cos(robotTheta) + robotY;
    double chord = calcDistance(predictedX, predictedY, targetX, targetY);
    
    double angleError = 2*asin( (chord / 2) / (radius) );
    
    predictedX = radius*sin( fmod(angleError + robotTheta, 2*M_PI) ) + robotX;
    predictedY = radius*cos( fmod(angleError + robotTheta, 2*M_PI) ) + robotY;
    
    if( (predictedX < targetX + 0.1) && (predictedX > targetX - 0.1) && (predictedY < targetY + 0.1) && (predictedY > targetY - 0.1) )
        return angleError;
    else
        return angleError*-1;
}

double calcAngleError(double targetX, double targetY, action direction)
{
    if(direction == rev)
    {
        double radius = calcDistance(robotX, robotY, targetX, targetY);
        double predictedX = radius*sin(robotTheta + degreesToRadians(180)) + robotX;
        double predictedY = radius*cos(robotTheta + degreesToRadians(180)) + robotY;
        double chord = calcDistance(predictedX, predictedY, targetX, targetY);

        double angleError = 2*asin( (chord / 2) / (radius) );

        predictedX = radius*sin( fmod(angleError + robotTheta + degreesToRadians(180), 2*M_PI) ) + robotX;
        predictedY = radius*cos( fmod(angleError + robotTheta + degreesToRadians(180), 2*M_PI) ) + robotY;

        if( (predictedX < targetX + 0.1) && (predictedX > targetX - 0.1) && (predictedY < targetY + 0.1) && (predictedY > targetY - 0.1) )
            return angleError;
        else
            return angleError*-1;
    }
    else return calcAngleError(targetX, targetY);
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

int auton_thread_Intake() 
{
  vex::task suspend();
  while(!autonStarted) vex::task::sleep(10);
  vex::task resume();
  
  while (true) 
  {
    if (intakeOn) 
    {
      if (intakeUp && !intakeDown) 
      {
        intake.spin(vex::directionType::fwd, 600, vex::velocityUnits::rpm);
        //intake2.spin(vex::directionType::fwd, 600, vex::velocityUnits::rpm);
      } else if (!intakeUp && intakeDown) 
      {
        intake.spin(vex::directionType::rev, 300, vex::velocityUnits::rpm);
        //intake2.spin(vex::directionType::rev, 600, vex::velocityUnits::rpm);
      } else if (!intakeUp && !intakeDown) 
      {
        intake.spin(vex::directionType::rev, 0, vex::velocityUnits::rpm);
        //intake2.spin(vex::directionType::rev, 0, vex::velocityUnits::rpm);
      }
    }
      
    vex::task::sleep(10);
  }
}

int armPos;

bool armUp = true;
int auton_thread_CapBaller() {
    capBaller.resetRotation();
    vex::task suspend();
    while(!autonStarted) vex::task::sleep(10);
    vex::task resume();

    double kArm = 1.5;
    double error;

    int count = 0;
    
    while(true) {
        if (!armUp) {

            error = armPot.value(vex::rotationUnits::deg) - armPos;

            if(error > 0)
              capBaller.spin(vex::directionType::fwd, abs(error*kArm), vex::velocityUnits::rpm);
            if (error < 0)
              capBaller.spin(vex::directionType::rev, abs(error*kArm), vex::velocityUnits::rpm);

            if(abs(error) < 2)
              count++;
            else count = 0;

            if(count > 50)
              capBaller.stop(vex::brakeType::hold);
        }
        else if (armUp) {
            capBaller.spin(vex::directionType::rev, 100,vex::velocityUnits::rpm);
            vex::task::sleep(500);
            capBaller.stop(vex::brakeType::coast);
            while(armUp) vex::task::sleep(5);
        }
        vex::task::sleep(5); 
    }
}

bool firing = false;

int auton_thread_Catapult()
{
    vex::task suspend();
    while(!autonStarted) vex::task::sleep(10);
    vex::task resume();
    
    int restPos = 2900;
    int ratchetError = 100;
    int error;
    double pSpeed;
    double kP = 1;
    while(true) 
    {
        if(!firing)
        {
            while(catapultPot.value(vex::analogUnits::range12bit) < restPos+ratchetError)
            {
                error = (restPos+ratchetError) - catapultPot.value(vex::analogUnits::range12bit);
                pSpeed = error*kP;
                catapult.spin(vex::directionType::rev,pSpeed,vex::velocityUnits::rpm);
                catapult2.spin(vex::directionType::rev,pSpeed,vex::velocityUnits::rpm);
            }
            while(catapultPot.value(vex::analogUnits::range12bit) > restPos && !firing)   
            {
                catapult.stop(vex::brakeType::coast);
                catapult2.stop(vex::brakeType::coast);
            }
        }
        else if(firing)
        {
            catapult.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);
            catapult2.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);
            vex::task::sleep(1000);
            firing = false;
        }
        vex::task::sleep(10);
    }
}

int encoderFilter() {
    while(true) {
        rightEncoderValue = (int) rightEncoder.rotation(vex::rotationUnits::raw)*-1;
        leftEncoderValue = (int) leftEncoder.rotation(vex::rotationUnits::raw);
        vex::task::sleep(10);
    }
}

int potentiometerFilter() {
    while(true) {
        potValue = armPot.value(vex::rotationUnits::deg);
        //potValue = armPot.value(vex::analogUnits::range12bit);
        vex::task::sleep(10);
    }
}

int sensorDebug() {
    while(true) {
        Brain.Screen.clearLine(1, vex::color::black);
        Brain.Screen.setCursor(1, 0);
        Brain.Screen.print("Potentiometer: %d", potValue);
        
        Brain.Screen.clearLine(2, vex::color::black);
        Brain.Screen.setCursor(2, 0);
        Brain.Screen.print("Left Encoder: %d", leftEncoderValue);
        
        Brain.Screen.clearLine(3, vex::color::black);
        Brain.Screen.setCursor(3, 0);
        Brain.Screen.print("Right Encoder: %d", rightEncoderValue);
        
        Brain.Screen.render();
        vex::task::sleep(10);
    }
}

int thread_Odometry()
{    
    leftEncoder.resetRotation();
    rightEncoder.resetRotation();
    vex::task::sleep(100);

    while(true)
    {
        currentLeft = inchesTraveled(leftEncoder.rotation(vex::rotationUnits::raw)); //read encoders
        currentRight = inchesTraveled(rightEncoder.rotation(vex::rotationUnits::raw)*-1);
        
        dLeftVal = (currentLeft - prevLeft);
        dRightVal = (currentRight - prevRight);
        
        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;
        
        dTheta = (dLeftVal - dRightVal) / ENCODER_WIDTH; //calculate change in angle in radians
        robotTheta += dTheta;
        robotTheta = fmod(robotTheta, 2*M_PI);
        if(robotTheta < 0) robotTheta += 2*M_PI;        
        
        dX = (dLeftVal + dRightVal)/2 * sin( (robotTheta) ); //calculate change in x
        dY = (dLeftVal + dRightVal)/2 * cos( (robotTheta) ); //calculate change in y
        
        robotX += dX; //add to current x and y
        robotY += dY;
        
        vex::task::sleep(DELAY); //reupdate every dT msec
    }
}

int display_Odometry()
{
    while(true)
    {
        Brain.Screen.clearScreen();
        
        Brain.Screen.setCursor(1,0);
        Brain.Screen.print("X: %f", robotX);
        
        Brain.Screen.setCursor(2,0);
        Brain.Screen.print("Y: %f", robotY);
        
        Brain.Screen.setCursor(3,0);
        Brain.Screen.print("Angle: %f", radiansToDegrees(robotTheta));
        
        Brain.Screen.setCursor(4,0);
        Brain.Screen.print("dLeft: %f", dLeftVal);
        
        Brain.Screen.setCursor(5,0);
        Brain.Screen.print("dRight: %f", dRightVal);
        
        Brain.Screen.setCursor(6,0);
        Brain.Screen.print("dX: %f", dX);
        
        Brain.Screen.setCursor(7,0);
        Brain.Screen.print("dY: %f", dY);
        
        Brain.Screen.setCursor(8,0);
        Brain.Screen.print("dTheta: %f", dTheta);
        
        Brain.Screen.setCursor(9,0);
        Brain.Screen.print("currentLeft: %f", currentLeft);
        
        Brain.Screen.setCursor(10,0);
        Brain.Screen.print("currentRight: %f", currentRight);
        
        /*Brain.Screen.setCursor(11,0);
        Brain.Screen.print(radiansToDegrees(calcAngleError(10,0)));*/
        
        Brain.Screen.render();
        vex::task::sleep(DELAY);
    }
}

//Auton Subfunctinos [Auton Only] =============================================

int intakeControl(action direction) {
  if (direction == up) {
    intakeUp = true;
    intakeDown = false;
  } else if (direction == down) {
    intakeUp = false;
    intakeDown = true;
  } else if (direction == stop) {
    intakeUp = false;
    intakeDown = false;
  }
  return 0;
}

void resetSlew()
{
    currentSpeed = 0;
}

void delay(int duration)
{
    vex::task::sleep(duration);
}

void spinLeft(vex::directionType dir, int rpm)
{
    leftDrive.spin(dir,rpm,vex::velocityUnits::rpm);
    leftDrive2.spin(dir,rpm,vex::velocityUnits::rpm);
}

void spinRight(vex::directionType dir, int rpm)
{
    rightDrive.spin(dir,rpm,vex::velocityUnits::rpm);
    rightDrive2.spin(dir,rpm,vex::velocityUnits::rpm);
}

void brake()
{
    leftDrive.stop(vex::brakeType::brake);
    leftDrive2.stop(vex::brakeType::brake);
    rightDrive.stop(vex::brakeType::brake);
    rightDrive2.stop(vex::brakeType::brake);
    delay(100);
}

//Auton Main Functions [Auton Only] ===============================================

void fire()
{
    firing = true;
}

void capBall(int pos)
{
    armUp = false;
    armPos = pos;
}

void capBall(action direction)
{
  if(direction == up)
    armUp = up;
}

void face(double targetTheta)
{
    targetTheta = degreesToRadians(targetTheta);
    
    const double MARGIN_OF_ERROR = 4.0;
    const double kP = 1.80;
    const double kI = 0.0;
    const double kD = 0.2;

    double thetaX = 100*sin(targetTheta) + robotX;
    double thetaY = 100*cos(targetTheta) + robotY;

    double PID_Speed;
    double proportional;
    double integral;
    double derivative;

    double angleError = calcAngleError(thetaX, thetaY);
    double prevAngleError = angleError;

    int accuracyCounter = 0;
    int msecAccurate = 100;
    int numOfAccurateIntervals = msecAccurate/DELAY;
    
    while(accuracyCounter < numOfAccurateIntervals)
    {
        thetaX = 100*sin(targetTheta) + robotX;
        thetaY = 100*cos(targetTheta) + robotY;
        
        angleError = calcAngleError(thetaX, thetaY);        
        
        proportional = radiansToDegrees( fabs(angleError) );

        if(proportional*kP < 200 && radiansToDegrees( fabs(angleError) )  != 0)
            integral += radiansToDegrees( fabs(angleError) );
        else
            integral = 0;
        
        derivative = angleError - prevAngleError;
        
        prevAngleError = angleError;

        PID_Speed = slew(proportional*kP + integral*kI + derivative*kD, 2.5);

        if(radiansToDegrees( fabs(angleError) ) <  MARGIN_OF_ERROR/2) accuracyCounter++;
        else accuracyCounter = 0;
        
        if(angleError < 0) //left
        {
            spinLeft(vex::directionType::rev, PID_Speed);
            spinRight(vex::directionType::fwd, PID_Speed);
        }
        else if(angleError > 0) //right
        {
            spinLeft(vex::directionType::fwd, PID_Speed);
            spinRight(vex::directionType::rev, PID_Speed);
        }
        
        vex::task::sleep(DELAY);
    }
    brake();
}

void face(double targetX, double targetY, action direction)
{    
    const double MARGIN_OF_ERROR = 4.0;
    const double kP = 1.80;
    const double kI = 0.0;
    const double kD = 0.25;

    double PID_Speed;
    double proportional;
    double integral;
    double derivative;

    double angleError = calcAngleError(targetX, targetY);
    double prevAngleError = angleError;

    int accuracyCounter = 0;
    int msecAccurate = 100;
    int numOfAccurateIntervals = msecAccurate/DELAY;
    
    while(accuracyCounter < numOfAccurateIntervals)
    {        
        angleError = calcAngleError(targetX, targetY, direction); 
        
        proportional = radiansToDegrees( fabs(angleError) );

        if(proportional*kP < 200 && radiansToDegrees( fabs(angleError) ) != 0)
            integral += radiansToDegrees( fabs(angleError) );
        else
            integral = 0;
        
        derivative = angleError - prevAngleError;
        
        prevAngleError = angleError;

        PID_Speed = proportional*kP + integral*kI + derivative*kD;

        if(radiansToDegrees( fabs(angleError) ) <  MARGIN_OF_ERROR/2) accuracyCounter++;
        else accuracyCounter = 0;
        
        if(angleError < 0) //left
        {
            spinLeft(vex::directionType::rev, PID_Speed);
            spinRight(vex::directionType::fwd, PID_Speed);
        }
        else if(angleError > 0) //right
        {
            spinLeft(vex::directionType::fwd, PID_Speed);
            spinRight(vex::directionType::rev, PID_Speed);
        }
        
        vex::task::sleep(DELAY);
    }
    brake();
}

void face(double targetX, double targetY, action direction, double partition)
{    
    const double MARGIN_OF_ERROR = 4.0;
    const double kP = 1.80;
    const double kI = 0.0;
    const double kD = 0.25;

    double PID_Speed;
    double proportional;
    double integral;
    double derivative;

    double angleError = calcAngleError(targetX, targetY);
    double prevAngleError = angleError;

    int accuracyCounter = 0;
    int msecAccurate = 100;
    int numOfAccurateIntervals = msecAccurate/DELAY;
    
    while(accuracyCounter < numOfAccurateIntervals)
    {        
        angleError = calcAngleError(targetX, targetY, direction); 
        
        proportional = radiansToDegrees( fabs(angleError) );

        if(proportional*kP < 200 && radiansToDegrees( fabs(angleError) ) != 0)
            integral += radiansToDegrees( fabs(angleError) );
        else
            integral = 0;
        
        derivative = angleError - prevAngleError;
        
        prevAngleError = angleError;

        PID_Speed = proportional*kP + integral*kI + derivative*kD;

        if(radiansToDegrees( fabs(angleError) ) <  MARGIN_OF_ERROR/2) accuracyCounter++;
        else accuracyCounter = 0;

        if(fabs(angleError) < partition)
        {
          capBall(25);
        }

        if(angleError < 0) //left
        {
            spinLeft(vex::directionType::rev, PID_Speed);
            spinRight(vex::directionType::fwd, PID_Speed);
        }
        else if(angleError > 0) //right
        {
            spinLeft(vex::directionType::fwd, PID_Speed);
            spinRight(vex::directionType::rev, PID_Speed);
        }
        
        vex::task::sleep(DELAY);
    }
    brake();
}


void drive(action direction, double targetDistance, double maxSpeed, bool brakeOn)
{
    double currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
    double currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
    double relative_theta = 0.0;
    double relative_dTheta; //calculate change in angle in radians
    
    double prevLeft = currentLeft; //update prev values
    double prevRight = currentRight;
    
    double dLeftVal = (currentLeft - prevLeft);
    double dRightVal = (currentRight - prevRight);
    
    double leftValue = 0.0;
    double rightValue = 0.0;
    
    double distanceError = targetDistance;
    double prevDistanceError = distanceError;
    double syncError;
    
    double distanceSpeed;
    double proportional;
    double derivative;
    double integral;
    double integralSpeed;
    double leftSpeed;
    double rightSpeed;
    double kSync = 2.0;
    double kI;
    double kP;
    double kD;
    
    
    if(direction == rev)
    {
        kI = 0.0;
        kP = 3.0;
        kD = 10.0;
    }
    else
    {
        kI = 0.1;
        kP = 11.0;
        kD = 3.0;
    }
    
    double drivePos;
    
    int delayCount = 0;
    int accuracyCounter = 0;
    int dT = DELAY;
    int milliseconds = 50;
    double accurate = milliseconds / dT;
    int initDelay = 50;
    
    if (direction == rev && targetDistance < 15) initDelay = 60;
    
    while(delayCount < initDelay) //initial delay to make sure both sides start on the same time
    {
        currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
        currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
        
        dLeftVal = currentLeft - prevLeft;
        dRightVal = currentRight - prevRight;
        
        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;
        
        relative_dTheta = radiansToDegrees((dLeftVal - dRightVal) / ENCODER_WIDTH); //calculate change in angle in radians, converted to degrees
        relative_theta += dTheta;
        
        leftValue += dLeftVal;
        rightValue += dRightVal;
        
        drivePos = (leftValue + rightValue)/2;
        
        distanceError = targetDistance - drivePos;
        syncError = relative_theta - 0.0;
        
        proportional = distanceError*kP;
        
        derivative = (distanceError - prevDistanceError)*kD;
        
        prevDistanceError = distanceError;
        
        if(proportional < 200 && distanceError != 0)
            integral += fabs(distanceError);
        else
            integral = 0;
        
        integralSpeed = integral * kI;
        
        if(integralSpeed > 100) integralSpeed = 100;

        distanceSpeed = slew(proportional, 2.5) + integralSpeed  + derivative;
        
        if(distanceSpeed > maxSpeed) distanceSpeed = maxSpeed;

        leftSpeed = distanceSpeed + syncError*kSync;
        rightSpeed = distanceSpeed - syncError*kSync;

        if(fabs(distanceError) < 10) accuracyCounter++;
        else accuracyCounter = 0;
        
        if(direction == fwd)
        {
            spinLeft(vex::directionType::fwd, leftSpeed);
        }
        else if(direction == rev)
        {
            spinRight(vex::directionType::rev, rightSpeed);
        }
        
        delay(dT);
        delayCount += dT;
    }
    
    while(fabs(drivePos) < targetDistance)
    {
        currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
        currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
        
        dLeftVal = currentLeft - prevLeft;
        dRightVal = currentRight - prevRight;
        
        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;
        
        relative_dTheta = radiansToDegrees((dLeftVal - dRightVal) / ENCODER_WIDTH); //calculate change in angle in radians, converted to degrees
        relative_theta += dTheta;
        
        leftValue += dLeftVal;
        rightValue += dRightVal;
        
        drivePos = (leftValue + rightValue)/2;
        
        distanceError = targetDistance - drivePos;
        syncError = relative_theta - 0.0;
        
        proportional = distanceError*kP;
        
        derivative = (distanceError - prevDistanceError)*kD;
        
        prevDistanceError = distanceError;
        
        if(proportional < 200 && distanceError != 0)
            integral += fabs(distanceError);
        else
            integral = 0;
        
        integralSpeed = integral * kI;
        
        if(integralSpeed > 100) integralSpeed = 100;

        distanceSpeed = slew(proportional, 2.5) + integralSpeed  + derivative;
        
        if(distanceSpeed > maxSpeed) distanceSpeed = maxSpeed;

        leftSpeed = distanceSpeed + syncError*kSync;
        rightSpeed = distanceSpeed - syncError*kSync;

        if(fabs(distanceError) < 10) accuracyCounter++;
        else accuracyCounter = 0;
        
        if(direction == fwd)
        {
            spinLeft(vex::directionType::fwd, leftSpeed);
            spinRight(vex::directionType::fwd, rightSpeed);
        }
        else if(direction == rev)
        {
            spinLeft(vex::directionType::rev, leftSpeed);
            spinRight(vex::directionType::rev, rightSpeed);
        }
        
        delay(dT);
    }
    
    if(brakeOn) brake();
    resetSlew();
}

void drive(action direction, double targetDistance, double maxSpeed, bool brakeOn, double partition)
{
    double currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
    double currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
    double relative_theta = 0.0;
    double relative_dTheta; //calculate change in angle in radians
    
    double prevLeft = currentLeft; //update prev values
    double prevRight = currentRight;
    
    double dLeftVal = (currentLeft - prevLeft);
    double dRightVal = (currentRight - prevRight);
    
    double leftValue = 0.0;
    double rightValue = 0.0;
    
    double distanceError = targetDistance;
    double prevDistanceError = distanceError;
    double syncError;
    
    double distanceSpeed;
    double proportional;
    double derivative;
    double integral;
    double integralSpeed;
    double leftSpeed;
    double rightSpeed;
    double kSync = 2.0;
    double kI;
    double kP;
    double kD;
    
    
    if(direction == rev)
    {
        kI = 0.0;
        kP = 3.0;
        kD = 10.0;
    }
    else
    {
        kI = 0.1;
        kP = 11.0;
        kD = 3.0;
    }
    
    double drivePos;
    
    int delayCount = 0;
    int accuracyCounter = 0;
    int dT = DELAY;
    int milliseconds = 50;
    double accurate = milliseconds / dT;
    int initDelay = 0;
    
    if(direction == fwd) initDelay = 30;
    else if (direction == rev) initDelay = 1000;

    while(delayCount < initDelay) //initial delay to make sure both sides start on the same time
    {
        currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
        currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
        
        dLeftVal = currentLeft - prevLeft;
        dRightVal = currentRight - prevRight;
        
        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;
        
        relative_dTheta = radiansToDegrees((dLeftVal - dRightVal) / ENCODER_WIDTH); //calculate change in angle in radians, converted to degrees
        relative_theta += dTheta;
        
        leftValue += dLeftVal;
        rightValue += dRightVal;
        
        drivePos = (leftValue + rightValue)/2;
        
        distanceError = targetDistance - drivePos;
        syncError = relative_theta - 0.0;
        
        proportional = distanceError*kP;
        
        derivative = (distanceError - prevDistanceError)*kD;
        
        prevDistanceError = distanceError;
        
        if(proportional < 200 && distanceError != 0)
            integral += fabs(distanceError);
        else
            integral = 0;
        
        integralSpeed = integral * kI;
        
        if(integralSpeed > 100) integralSpeed = 100;

        distanceSpeed = slew(proportional, 2.5) + integralSpeed  + derivative;
        
        if(distanceSpeed > maxSpeed) distanceSpeed = maxSpeed;

        leftSpeed = distanceSpeed + syncError*kSync;
        rightSpeed = distanceSpeed - syncError*kSync;

        if(fabs(distanceError) < 10) accuracyCounter++;
        else accuracyCounter = 0;
        
        if(direction == fwd)
        {
            spinLeft(vex::directionType::fwd, leftSpeed);
        }
        else if(direction == rev)
        {
            spinRight(vex::directionType::rev, rightSpeed);
        }
        
        delay(dT);
        delayCount += dT;
    }
    
    
    while(fabs(drivePos) < targetDistance)
    {
        currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
        currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
        
        dLeftVal = currentLeft - prevLeft;
        dRightVal = currentRight - prevRight;
        
        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;
        
        relative_dTheta = radiansToDegrees((dLeftVal - dRightVal) / ENCODER_WIDTH); //calculate change in angle in radians, converted to degrees
        relative_theta += dTheta;
        
        leftValue += dLeftVal;
        rightValue += dRightVal;
        
        drivePos = (leftValue + rightValue)/2;
        
        distanceError = targetDistance - drivePos;
        syncError = relative_theta - 0.0;
        
        proportional = distanceError*kP;
        
        derivative = (distanceError - prevDistanceError)*kD;
        
        prevDistanceError = distanceError;
        
        if(proportional < 200 && distanceError != 0)
            integral += fabs(distanceError);
        else
            integral = 0;
        
        integralSpeed = integral * kI;
        
        if(integralSpeed > 100) integralSpeed = 100;

        distanceSpeed = slew(proportional, 2.5) + integralSpeed  + derivative;
        
        if(distanceSpeed > maxSpeed) distanceSpeed = maxSpeed;

        leftSpeed = distanceSpeed + syncError*kSync;
        rightSpeed = distanceSpeed - syncError*kSync;

        if(fabs(distanceError) < 10) accuracyCounter++;
        else accuracyCounter = 0;

        if(rightValue > partition || leftValue > partition) {
            leftSpeed = 80;
            rightSpeed = 80;
        }
        
        if(direction == fwd)
        {
            spinLeft(vex::directionType::fwd, leftSpeed);
            spinRight(vex::directionType::fwd, rightSpeed);
        }
        else if(direction == rev)
        {
            spinLeft(vex::directionType::rev, leftSpeed);
            spinRight(vex::directionType::rev, rightSpeed);
        }
        
        delay(dT);
    }
    
    if(brakeOn) brake();
    resetSlew();
}

void flipDrive(action direction, double targetDistance, double maxSpeed, bool brakeOn, double partition)
{
    double currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
    double currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
    double relative_theta = 0.0;
    double relative_dTheta; //calculate change in angle in radians
    
    double prevLeft = currentLeft; //update prev values
    double prevRight = currentRight;
    
    double dLeftVal = (currentLeft - prevLeft);
    double dRightVal = (currentRight - prevRight);
    
    double leftValue = 0.0;
    double rightValue = 0.0;
    
    double distanceError = targetDistance;
    double prevDistanceError = distanceError;
    double syncError;
    
    double distanceSpeed;
    double proportional;
    double derivative;
    double integral;
    double integralSpeed;
    double leftSpeed;
    double rightSpeed;
    double kSync = 2.0;
    double kI;
    double kP;
    double kD;
    
    
    if(direction == rev)
    {
        kI = 0.0;
        kP = 3.0;
        kD = 10.0;
    }
    else
    {
        kI = 0.1;
        kP = 11.0;
        kD = 3.0;
    }
    
    double drivePos;
    
    int delayCount = 0;
    int accuracyCounter = 0;
    int dT = DELAY;
    int milliseconds = 50;
    double accurate = milliseconds / dT;
    int initDelay = 0;
    
    if(direction == fwd) initDelay = 30;
    else if (direction == rev) initDelay = 30;

    while(delayCount < initDelay) //initial delay to make sure both sides start on the same time
    {
        currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
        currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
        
        dLeftVal = currentLeft - prevLeft;
        dRightVal = currentRight - prevRight;
        
        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;
        
        relative_dTheta = radiansToDegrees((dLeftVal - dRightVal) / ENCODER_WIDTH); //calculate change in angle in radians, converted to degrees
        relative_theta += dTheta;
        
        leftValue += dLeftVal;
        rightValue += dRightVal;
        
        drivePos = (leftValue + rightValue)/2;
        
        distanceError = targetDistance - drivePos;
        syncError = relative_theta - 0.0;
        
        proportional = distanceError*kP;
        
        derivative = (distanceError - prevDistanceError)*kD;
        
        prevDistanceError = distanceError;
        
        if(proportional < 200 && distanceError != 0)
            integral += fabs(distanceError);
        else
            integral = 0;
        
        integralSpeed = integral * kI;
        
        if(integralSpeed > 100) integralSpeed = 100;

        distanceSpeed = slew(proportional, 2.5) + integralSpeed  + derivative;
        
        if(distanceSpeed > maxSpeed) distanceSpeed = maxSpeed;

        leftSpeed = distanceSpeed + syncError*kSync;
        rightSpeed = distanceSpeed - syncError*kSync;

        if(fabs(distanceError) < 10) accuracyCounter++;
        else accuracyCounter = 0;
        
        if(direction == fwd)
        {
            spinLeft(vex::directionType::fwd, leftSpeed);
        }
        else if(direction == rev)
        {
            spinRight(vex::directionType::rev, rightSpeed);
        }
        
        delay(dT);
        delayCount += dT;
    }
    
    
    while(fabs(drivePos) < targetDistance)
    {
        currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
        currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
        
        dLeftVal = currentLeft - prevLeft;
        dRightVal = currentRight - prevRight;
        
        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;
        
        relative_dTheta = radiansToDegrees((dLeftVal - dRightVal) / ENCODER_WIDTH); //calculate change in angle in radians, converted to degrees
        relative_theta += dTheta;
        
        leftValue += dLeftVal;
        rightValue += dRightVal;
        
        drivePos = (leftValue + rightValue)/2;
        
        distanceError = targetDistance - drivePos;
        syncError = relative_theta - 0.0;
        
        proportional = distanceError*kP;
        
        derivative = (distanceError - prevDistanceError)*kD;
        
        prevDistanceError = distanceError;
        
        if(proportional < 200 && distanceError != 0)
            integral += fabs(distanceError);
        else
            integral = 0;
        
        integralSpeed = integral * kI;
        
        if(integralSpeed > 100) integralSpeed = 100;

        distanceSpeed = slew(proportional, 2.5) + integralSpeed  + derivative;
        
        if(distanceSpeed > maxSpeed) distanceSpeed = maxSpeed;

        leftSpeed = distanceSpeed + syncError*kSync;
        rightSpeed = distanceSpeed - syncError*kSync;

        if(fabs(distanceError) < 10) accuracyCounter++;
        else accuracyCounter = 0;

        if(fabs(rightValue) > partition || fabs(leftValue) > partition) {
            capBall(up);
        }
        
        if(direction == fwd)
        {
            spinLeft(vex::directionType::fwd, leftSpeed);
            spinRight(vex::directionType::fwd, rightSpeed);
        }
        else if(direction == rev)
        {
            spinLeft(vex::directionType::rev, leftSpeed);
            spinRight(vex::directionType::rev, rightSpeed);
        }
        
        delay(dT);
    }
    
    if(brakeOn) brake();
    resetSlew();
}


void delayDrive(action direction, int duration, int speed, bool brakeOn) {
    
    if(direction == fwd)
    {
        spinLeft(vex::directionType::fwd, speed);
        spinRight(vex::directionType::fwd, speed);
    }
    
    if(direction == rev)
    {
        spinLeft(vex::directionType::rev, speed);
        spinRight(vex::directionType::rev, speed);
    }
    
    delay(duration);
    
    if(brakeOn) brake();
    resetSlew();
}


void delayCurveDrive(action direction, int duration, int leftSpeed, int rightSpeed, bool brakeOn)
{
    if(direction == fwd)
    {
        spinLeft(vex::directionType::fwd, leftSpeed);
        spinRight(vex::directionType::fwd, rightSpeed);
        delay(duration);
    }
    
    if(direction == rev)
    {
        spinLeft(vex::directionType::rev, rightSpeed);
        spinRight(vex::directionType::rev, leftSpeed);
        delay(duration);
    }
        
    if(brakeOn) brake();
}

void ghettoDrive(action direction, int distance, double speed, bool brakeOn)
{
  double currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
  double currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
  
  double prevLeft = currentLeft; //update prev values
  double prevRight = currentRight;
  
  double dLeftVal = (currentLeft - prevLeft);
  double dRightVal = (currentRight - prevRight);
  
  double leftValue = 0.0;
  double rightValue = 0.0;

  double drivePos = 0.0;


  while(fabs(drivePos) < distance)
  {
    currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
    currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
    
    dLeftVal = currentLeft - prevLeft;
    dRightVal = currentRight - prevRight;
    
    prevLeft = currentLeft; //update prev values
    prevRight = currentRight;
    
    leftValue += dLeftVal;
    rightValue += dRightVal;
    
    drivePos = (leftValue + rightValue)/2;

    if(direction == fwd)
    {
        spinLeft(vex::directionType::fwd, speed);
        spinRight(vex::directionType::fwd, speed);
    }
    else if(direction == rev)
    {
        spinLeft(vex::directionType::rev, speed);
        spinRight(vex::directionType::rev, speed);
    }

    delay(DELAY);
  }

  if(brakeOn) brake();
}

void ghettoFlipDrive(action direction, int distance, double speed, bool brakeOn, double partition)
{
  double currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
  double currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
  
  double prevLeft = currentLeft; //update prev values
  double prevRight = currentRight;
  
  double dLeftVal = (currentLeft - prevLeft);
  double dRightVal = (currentRight - prevRight);
  
  double leftValue = 0.0;
  double rightValue = 0.0;

  double drivePos = 0.0;


  while(fabs(drivePos) < distance)
  {
    currentLeft = (leftEncoder.rotation(vex::rotationUnits::raw)/ 360)*WHEEL_CIRCUMFERENCE; //read encoders
    currentRight = (rightEncoder.rotation(vex::rotationUnits::raw)*-1/ 360)*WHEEL_CIRCUMFERENCE;
    
    dLeftVal = currentLeft - prevLeft;
    dRightVal = currentRight - prevRight;
    
    prevLeft = currentLeft; //update prev values
    prevRight = currentRight;
    
    leftValue += dLeftVal;
    rightValue += dRightVal;
    
    drivePos = (leftValue + rightValue)/2;

    if(fabs(drivePos) > partition) {
        capBall(up);
    }

    if(direction == fwd)
    {
        spinLeft(vex::directionType::fwd, speed);
        spinRight(vex::directionType::fwd, speed);
    }
    else if(direction == rev)
    {
        spinLeft(vex::directionType::rev, speed);
        spinRight(vex::directionType::rev, speed);
    }

    delay(5);
  }

  if(brakeOn) brake();
}

void pre_auton(void) {
  int xPos;
  int yPos;
  bool sensorScreen = false;
  gyroCalibrated = true;
  vex::task encoderValues(encoderFilter);
  vex::task potValues(potentiometerFilter);
  vex::task sensorValues(sensorDebug);
    
  Brain.Screen.clearScreen();
  
  while (!autonStarted) { //GUI

    if(!sensorScreen)
    {
        sensorValues.stop();
        Brain.Screen.drawRectangle(bbx, bby, buttonWidth, buttonHeight, vex::color::blue);
        Brain.Screen.drawRectangle(rbx, rby, buttonWidth, buttonHeight, vex::color::red);
        Brain.Screen.drawRectangle(bfx, bfy, buttonWidth, buttonHeight, vex::color::blue);
        Brain.Screen.drawRectangle(rfx, rfy, buttonWidth, buttonHeight, vex::color::red);
        Brain.Screen.drawRectangle(sx, sy, buttonWidth, buttonHeight, vex::color::yellow);
        Brain.Screen.drawRectangle(tx, ty, buttonWidth, buttonHeight, vex::color::white);
        Brain.Screen.drawRectangle(dx, dy, 40, 40, vex::color::purple);
    }
    else
    {
        vex::task sensorValues(sensorDebug);
        Brain.Screen.drawRectangle(dx, dy, 40, 40, vex::color::purple);
    }

    Brain.Screen.render();

    if (Brain.Screen.pressing()) {
      xPos = Brain.Screen.xPosition();
      yPos = Brain.Screen.yPosition();

      if ((dx < xPos) && (xPos < (dx + buttonWidth)) && (dy < yPos) && (yPos < (dy + buttonHeight))) {
        sensorScreen = !sensorScreen;
        while (Brain.Screen.pressing())
           delay(10);
      }
      if(!sensorScreen)
      {
          //Red Front
          if ((rfx < xPos) && (xPos < (rfx + buttonWidth)) && (rfy < yPos) && (yPos < (rfy + buttonHeight))) {
            autonNum = 1;
          }
          //Blue Front
          if ((bfx < xPos) && (xPos < (bfx + buttonWidth)) && (bfy < yPos) && (yPos < (bfy + buttonHeight))) {
            autonNum = 2;
          }
          //Red Back
          if ((rbx < xPos) && (xPos < (rbx + buttonWidth)) && (rby < yPos) && (yPos < (rby + buttonHeight))) {
            autonNum = 3;
          }
          //Blue Back
          if ((bbx < xPos) && (xPos < (bbx + buttonWidth)) && (bby < yPos) && (yPos < (bby + buttonHeight))) {
            autonNum = 4;
          }
          //skills
          if ((sx < xPos) && (xPos < (sx + buttonWidth)) && (sy < yPos) && (yPos < (sy + buttonHeight))) {
            autonNum = 5;
          }
          //test
          if ((tx < xPos) && (xPos < (tx + buttonWidth)) && (ty < yPos) && (yPos < (ty + buttonHeight))) {
            autonNum = 6;
          }
      }
    }

    Brain.Screen.clearScreen();
    switch (autonNum) {
    default: Brain.Screen.printAt(150, 20, "Autonomous: None");
      break;
    case 1:
        Brain.Screen.printAt(150, 20, "Autonomous: Red Front");
      break;
    case 2:
        Brain.Screen.printAt(150, 20, "Autonomous: Blue Front");
      break;
    case 3:
        Brain.Screen.printAt(150, 20, "Autonomous: Anti-Red Counter");
      break;
    case 4:
        Brain.Screen.printAt(150, 20, "Autonomous: Anti-Blue Counter");
      break;
    case 5:
        Brain.Screen.printAt(150, 20, "Autonomous: Skills"); 
      break;
    case 6:
        Brain.Screen.printAt(150, 20,"Autonomous: Test"); 
      break;
    } 
  }
  sensorValues.stop();
  encoderValues.stop();
  potValues.stop();
}
  vex::task auton_01 (auton_thread_Intake);
  vex::task auton_02 (auton_thread_Catapult);
  vex::task auton_03 (auton_thread_CapBaller);
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous(void) {

  Brain.Screen.clearScreen();  
  vex::task positionTracking(thread_Odometry);
  //vex::task displayPosition(display_Odometry);
  
  vex::task::sleep(150);
  autonStarted = true;
  Brain.Screen.clearScreen();  
  switch (autonNum) {

      case 1: //front red

      Brain.resetTimer();

        robotTheta = degreesToRadians(25);

        intakeControl(up);

        drive(fwd, calcDistance(robotX, robotY, 13, 20)-8, 200, true);

        capBall(48);

        //delay(250);

        face(0,0, rev);

        //drive(rev, calcDistance(robotX, robotY, 0,0)-3, 200, true);

        ghettoFlipDrive(rev, calcDistance(robotX, robotY, 0,0)-8, 200, true,5);

        delay(500);

        capBall(up);

        face(-100,8,rev);
        
        fire(); //hit top flag

        vex::task::sleep(150);
          
        face(-49,0,rev); //face bottom flag
          
        intakeControl(stop);
          
        drive(rev, calcDistance(robotX,robotY,-42,-5), 200, true); //hit bottom flag

        face(0,2, fwd);

        drive(fwd, calcDistance(robotX, robotY, 0,0)-3, 200, true);

        face(-4,45, fwd); //face middle cap

        intakeControl(up);

        drive(fwd, calcDistance(robotX,robotY, -2, 45) - 6, 200, true, calcDistance(robotX,robotY, -2, 45) - 25);

        ghettoDrive(rev, 5, 200, true);

        delay(250);

        //capBall(25);

        face(-28,20,fwd,5); //face cap with 2 balls

        flipDrive(fwd, calcDistance(robotX,robotY,-24,21)-12, 200, true, calcDistance(robotX,robotY,-24,21)-20);

        delay(250);

        face(-50, 43.5, rev);

        while(Brain.timer(vex::timeUnits::msec) < 14000)
            delay(10);
          
        fire();
          
        break;
          
     case 2: //front blue

     Brain.resetTimer();
          
        robotTheta = degreesToRadians(335);

        intakeControl(up);

        drive(fwd, calcDistance(robotX, robotY, -13, 20)-10, 200, true);

        capBall(48);

        face(0,0, rev);

        //flipDrive(rev, calcDistance(robotX, robotY, 5,0) - 3, 200, true, 5);

        ghettoFlipDrive(rev, calcDistance(robotX, robotY, 0,0)-5, 200, true , 5);

        delay(500);

        //capBall(up);

        face(100,5,rev);
        
        fire();

        delay(100);
          
        face(49,2,rev); //face bottom flag
          
        intakeControl(stop);
          
        drive(rev, calcDistance(robotX,robotY,42,-5) + 2, 200, true); //hit bottom flag

        //face(0,0, fwd);

        delay(250);

        intakeControl(down);

        drive(fwd, calcDistance(robotX, robotY, 0,0)-2, 200, true);

        face(3,48, fwd); //face middle cap

        intakeControl(up);

        drive(fwd, calcDistance(robotX,robotY, 2, 45) - 6, 200, true, calcDistance(robotX,robotY, -2, 45) - 25);

        ghettoDrive(rev, 5, 200, true);

        delay(250);

        //capBall(25);

        face(20,24,fwd,5);

        flipDrive(fwd, calcDistance(robotX,robotY,-24,21)-10, 200, true, calcDistance(robotX,robotY,-24,21)-19);

        delay(250);

        face(50, 44, rev);

        while(Brain.timer(vex::timeUnits::msec) < 14000)
            delay(10);
          
        fire();

        delay(1000);
          
        break;

      case 3: //counter red
          
        Brain.resetTimer();
          
        intakeControl(up);  

        drive(fwd, 30, 200, true);
          
        drive(fwd, 5, 200, true);
          
        drive(rev, calcDistance(robotX,robotY,0,2), 200, true);
          
        face(-48,-2,rev);
          
        drive(rev, calcDistance(robotX,robotY,-48,-1), 200, true);
          
        drive(fwd, calcDistance(robotX,robotY,-8,0), 200, true);
          
        face(-48,1, rev);
          
        intakeControl(stop);
          
        while(Brain.timer(vex::timeUnits::msec) < 14400)
            delay(10);
          
        fire();

        delay(1000);

        break;
          
      case 4: //counter blue
          
        Brain.resetTimer();
          
        intakeControl(up);  

        drive(fwd, 30, 200, true);
          
        drive(fwd, 5, 200, true);
          
        drive(rev, calcDistance(robotX,robotY,0,0), 200, true);
          
        face(48,-2,rev);
          
        drive(rev, calcDistance(robotX,robotY,48,-1), 200, true);
          
        drive(fwd, calcDistance(robotX,robotY,9,0), 200, true);
          
        face(48, 2, rev);
          
        intakeControl(stop);
          
        while(Brain.timer(vex::timeUnits::msec) < 14400)
            delay(10);
          
        fire();
        
        break;   
          
      case 5: //skills

        capBall(58);
          
        delay(500);
        
        intakeControl(up);
          
        drive(fwd, 43, 200, true); //pick up first ball, flip first cap
          
        vex::task::sleep(500);
          
        drive(rev, calcDistance(robotX,robotY,robotX,24), 200, true);
          
        face(30,48,fwd);
          
        capBall(20);

        intakeControl(stop);
          
        drive(fwd, calcDistance(robotX,robotY,22,40)-13, 150, true); //flip second cap

        delay(250);
          
        capBall(up);

        delay(500);
          
        face(0,0,rev);
          
        drive(rev, calcDistance(robotX,robotY,0,0), 200, true);
          
        face(94); //-100,0,rev
          
        drive(rev, calcDistance(robotX,robotY,-50,robotY), 200, true);
          
        face(-100,3,rev); //face first flags
          
        delay(250);
          
        fire(); //shoot both flags
          
        delay(150);
          
        face(-100,-4, rev);
          
        drive(rev, calcDistance(robotX,robotY,-96,-2) - 2, 200, true); //hit bottom flag

        drive(fwd, calcDistance(robotX,robotY,-50,robotY)-7, 200, true);

        face(-50,48,fwd); //face middle cap
          
        intakeControl(up);
          
        capBall(57);

        delay(1000);

        drive(fwd, calcDistance(robotX,robotY,-48,48)-13, 200, true); //flip cap pick ball
          
        delay(250);

        ghettoDrive(rev,7,100,true);

        face(-60,24,fwd); //face middle double cap

        capBall(20);

        flipDrive(fwd, calcDistance(robotX,robotY,-66,24)-4,200,true,calcDistance(robotX,robotY,-66,24)-9); //flip double cap get ball

        delay(250);

        face(-56,48,rev);

        drive(rev, calcDistance(robotX,robotY,-54,48)-10,200,true);

        face(-100,45,rev);

        fire(); //shoot middle flags

        delay(250);

        face(-100,37, rev);

        drive(rev, calcDistance(robotX,robotY,-100,40)-5, 200, true); //flip bottom middle

        drive(fwd, calcDistance(robotX,robotY,-68,robotY)-8, 200, true);
          
        face(-89, 96, fwd); //face far cap

        capBall(28);

        flipDrive(fwd,calcDistance(robotX,robotY,-74,96)-5,200, true, calcDistance(robotX,robotY,-74,96)-10);

        delay(500);

        face(-120,89, rev);
          
        drive(rev, calcDistance(robotX,robotY,-120,103)-13, 200, true); //flip far bottom flag
          
        face(-46,104,fwd);
          
        drive(fwd, calcDistance(robotX,robotY,-46,98)-23, 200, true);

        face(-53, 72, fwd); //face far cap with ball under

        capBall(58);

        intakeControl(up);

        delay(500);

        ghettoDrive(fwd, calcDistance(robotX, robotY, -54, 72)-17, 200, true); //pick up ball flip cap

        delay(500);

        capBall(up);

        drive(rev, calcDistance(robotX, robotY, -54, 96), 200, true);

        intakeControl(stop);

        face(-84,97,rev);

        fire(); //shoot two far flgas

        delay(250);
        //==============================
        face(-30,120,rev);

        drive(rev, calcDistance(robotX,robotY,-30,120)-6, 200, true);

        face(-24,0,fwd); //face platform

        intakeControl(up);

        drive(fwd, 52, 200, true);
        //===============================
        /*face(53); //about to go to the back

        drive(fwd,48,200, true);

        face(25,82,fwd); //face far far cap with double ball

        capBall(25);

        drive(fwd, calcDistance(robotX,robotY,26,72)-27 , 200,true);

        capBall(up); //flip far far cap

        delay(500);

        face(-9,110,rev);

        drive(rev, calcDistance(robotX,robotY,-9,112), 200, true);

        face(-5, 72, fwd);

        capBall(58); //flip cap with ball under

        drive(fwd, calcDistance(robotX,robotY,0,72)-17, 200, true);

        delay(250);

        capBall(up);

        drive(rev, calcDistance(robotX,robotY,robotX,120)-2, 200, true); 

        face(-30,120,rev);

        drive(rev, calcDistance(robotX,robotY,-35,120)+1, 200, true);

        face(-24,0,fwd); //face platform

        intakeControl(up);

        drive(fwd, 52, 200, true);*/
          
        break;
          
      case 6:
          
        face(90);
        break;
  }
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

int drive_thread_Chassis()
{
    bool driveMode = true;
    int brakeDelay = 0;
    int maxSpeed = 100;
    double turnMultiplier = 1;
    while(true)
    {
        if ((abs(Controller1.Axis2.value()) > 10) && (abs(Controller1.Axis3.value()) > 10)) 
        {
            brakeDelay = 0;
            /*if (Controller1.ButtonR2.pressing()) 
            {
                if (Controller1.Axis2.value() > 0 && Controller1.Axis3.value() > 0) {
                    rightSpeed = 30;
                    leftSpeed = 30;
                }
                else if (Controller1.Axis2.value() < 0 && Controller1.Axis3.value() > 0) {
                    rightSpeed = -30;
                    leftSpeed = 30;
                }
                else if (Controller1.Axis2.value() > 0 && Controller1.Axis3.value() < 0) {
                    rightSpeed = 30;
                    leftSpeed = -30;
                }
                else if (Controller1.Axis2.value() < 0 && Controller1.Axis3.value() < 0) {
                    rightSpeed = (-30);
                    leftSpeed = (-30);
                }
            }*/
            if( (Controller1.Axis2.value() > 10 && Controller1.Axis3.value() < -10) || ((Controller1.Axis2.value() < -10 && Controller1.Axis3.value() > 10)) )
            {
                rightSpeed = Controller1.Axis2.value()*turnMultiplier;
                leftSpeed = Controller1.Axis3.value()*turnMultiplier;
            }
            else {
                rightSpeed = Controller1.Axis2.value();
                leftSpeed = rightSpeed;
                currentSpeedLeft = rightSpeed;
            }
        }
        
        else if ((abs(Controller1.Axis2.value()) < 10) && (abs(Controller1.Axis3.value()) > 10)) 
        {
            brakeDelay = 0;
            /*if (Controller1.ButtonR2.pressing()) 
            {
                if (Controller1.Axis3.value() > 0) {
                    rightSpeed = 0;
                    leftSpeed = 30;
                }
                else if (Controller1.Axis3.value() < 0) {
                    rightSpeed = 0;
                    leftSpeed = -30;
                }
            }*/
            //else {
                rightSpeed = (0);
                leftSpeed = (Controller1.Axis3.value()*turnMultiplier);
            //}
        }
        
        else if ((abs(Controller1.Axis2.value()) > 10) && (abs(Controller1.Axis3.value()) < 10)) 
        {
            brakeDelay = 0;
            /*if (Controller1.ButtonR2.pressing()) 
            {
                if (Controller1.Axis2.value() > 0) {
                    rightSpeed = (30);
                    leftSpeed = (0);
                }
                else if (Controller1.Axis2.value() < 0) {
                    rightSpeed = (-30);
                    leftSpeed = (0);
                }
            }*/
            //else {
                rightSpeed = (Controller1.Axis2.value()*turnMultiplier);
                leftSpeed = (0);
           //}
        }
        
        else if ((abs(Controller1.Axis2.value()) < 10) && (abs(Controller1.Axis3.value()) < 10)) 
        {
            brakeDelay+=20;
            rightSpeed = (0);
            leftSpeed = (0);
        }
               
        if(Controller1.ButtonA.pressing())
        {
            driveMode = true;
        }
        else if(Controller1.ButtonX.pressing())
        {
            driveMode = false;         
        }
                if(rightSpeed > maxSpeed) rightSpeed = maxSpeed;
                else if(rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;
                if(leftSpeed > maxSpeed) leftSpeed = maxSpeed;
                else if(leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;
        
        if(brakeDelay > 500)
        {
           leftDrive.stop(vex::brakeType::hold);
           leftDrive2.stop(vex::brakeType::hold);
           rightDrive.stop(vex::brakeType::hold);
           rightDrive2.stop(vex::brakeType::hold);
        }
        else
        {
           leftDrive.stop(vex::brakeType::coast);
           leftDrive2.stop(vex::brakeType::coast);
           rightDrive.stop(vex::brakeType::coast);
           rightDrive2.stop(vex::brakeType::coast);
        }
        
        if(driveMode)
        {
            leftDrive.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
            leftDrive2.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
            rightDrive.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
            rightDrive2.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
        }
        
        else if(!driveMode)
        {
            leftDrive.spin(vex::directionType::rev, rightSpeed, vex::velocityUnits::pct);
            leftDrive2.spin(vex::directionType::rev, rightSpeed, vex::velocityUnits::pct);
            rightDrive.spin(vex::directionType::rev, leftSpeed, vex::velocityUnits::pct);
            rightDrive2.spin(vex::directionType::rev, leftSpeed, vex::velocityUnits::pct);
        }
        
        vex::task::sleep(20);
    }
}

int drive_thread_Catapult()
{
    int restPos = 2900;
    int ratchetError = 100;
    int error;
    double pSpeed;
    int accuracyCounter = 0;
    double kP = 1;
    while(true) 
    {
        if(!firing)
        {
            while(catapultPot.value(vex::analogUnits::range12bit) < restPos+ratchetError)
            {
                error = (restPos+ratchetError) - catapultPot.value(vex::analogUnits::range12bit);
                pSpeed = error*kP;
                catapult.spin(vex::directionType::rev,pSpeed,vex::velocityUnits::rpm);
                catapult2.spin(vex::directionType::rev,pSpeed,vex::velocityUnits::rpm);
            }
            while(catapultPot.value(vex::analogUnits::range12bit) > restPos && !firing)   
            {
                catapult.stop(vex::brakeType::coast);
                catapult2.stop(vex::brakeType::coast);
            }
        }
        else if(firing)
        {
            catapult.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);
            catapult2.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);
            delay(1000);
            firing = false;
        }
        delay(10);
    }
}

int drive_thread_CatapultControl()
{
    while(true) 
    {
        if(Controller1.ButtonR1.pressing())
        {
            firing = true;
            
        }
        vex::task::sleep(10);
    }
}

int drive_thread_CapBaller() {
    capBaller.resetRotation();

    double kArm = 1.3;
    double error;

    int count = 0;
    
    while(true) {
        if (!armUp) {

            error = armPot.value(vex::rotationUnits::deg) - armPos;

            if(error > 0)
              capBaller.spin(vex::directionType::fwd, fabs(error*kArm), vex::velocityUnits::rpm);
            if (error < 0)
              capBaller.spin(vex::directionType::rev, fabs(error*kArm), vex::velocityUnits::rpm);

            if(fabs(error) < 2)
              count++;
            else count = 0;

            if(count > 50)
              capBaller.stop(vex::brakeType::hold);
        }
        else if (armUp) {
            capBaller.spin(vex::directionType::rev, 100,vex::velocityUnits::rpm);
            vex::task::sleep(500);
            capBaller.stop(vex::brakeType::coast);
            while(armUp) vex::task::sleep(5);
        }
        vex::task::sleep(5); 
    }
}

int drive_thread_CapBallerControl() {
    capBaller.resetRotation();
    delay(25);

    while(true) {
        if (Controller1.ButtonUp.pressing()) {
            armUp = false;
            armPos = 30;
            delay(1250);  
            armUp = true; 
            delay(500);
        }
        else if (Controller1.ButtonRight.pressing()) {
            armUp = false;
            armPos = 55;
            delay(1250);

            armPos = 65;

            delay(750);  
            armUp = true; 
            delay(500);            
        }
        else if (Controller1.ButtonDown.pressing()) {
            capBaller.spin(vex::directionType::rev, 100,vex::velocityUnits::rpm);
            armUp = true;
            delay(500);
        }
        else if(Controller1.ButtonR2.pressing())
        {
          armUp = !armUp;
          armPos = 30;

          while(Controller1.ButtonR2.pressing())
          {
            delay(10);
          }
        }
       // if(armUp) capBaller.stop(vex::brakeType::coast);
        //else capBaller.stop(vex::brakeType::hold);
        delay(10);
    }
}

int drive_thread_IntakeControl() 
{
  //int jammed = 0;
  while (true) 
  {
    if (intakeOn) 
    {
        if (Controller1.ButtonL1.pressing()) 
      {
        if (intakeUp) {
          intakeUp = false;
          intakeDown = false;
        } 
        else {
          intakeUp = true;
          intakeDown = false;
        }
        while (Controller1.ButtonL1.pressing())
          vex::task::sleep(10);
      } 
        
      else if (Controller1.ButtonL2.pressing()) 
      {
        if (intakeDown) {
          intakeUp = false;
          intakeDown = false;
        } 
        else {
          intakeUp = false;
          intakeDown = true;
        }
        while (Controller1.ButtonL2.pressing())
          vex::task::sleep(10);
      }  
      
      else {
        vex::task::sleep(10);
      }
      
      /*if ( fabs(intake.velocity(vex::velocityUnits::rpm) < 1)  && (intakeUp || intakeDown))
        jammed++;
      else jammed = 0;
      
      if (jammed > 50)
      {
        intakeUp = false;
        intakeDown = false;
      }*/
    }
  }
}
       
int drive_thread_Intake()
{
    while(true)
    {
    vex::task::sleep(10);
    if(intakeOn)
    {
      if (catapultPot.value(vex::analogUnits::range12bit) < 2900 && !intakeDown) {
        intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        //intake2.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
      } else if (intakeUp && !intakeDown) {
        intake.spin(vex::directionType::fwd, 600, vex::velocityUnits::rpm);
        //intake2.spin(vex::directionType::fwd, 400, vex::velocityUnits::rpm);
      } else if (!intakeUp && intakeDown) {
        intake.spin(vex::directionType::rev, 600, vex::velocityUnits::rpm);
        //intake2.spin(vex::directionType::rev, 600, vex::velocityUnits::rpm);
      } else {
        intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        //intake2.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
      }
      
    } else {
      vex::task::sleep(10);
    }
  }
}

void usercontrol(void) {
  auton_01.stop();
  auton_02.stop();
  auton_03.stop();
    
  intakeUp = true;
  intakeDown = false;
  
  vex::task driverControl_0(drive_thread_Chassis);
  vex::task driverControl_1(drive_thread_Intake);
  vex::task driverControl_2(drive_thread_IntakeControl);
  vex::task driverControl_3(drive_thread_Catapult);
  vex::task driverControl_4(drive_thread_CatapultControl);
  vex::task driverControl_5(drive_thread_CapBaller);
  vex::task driverControl_6(drive_thread_CapBallerControl);
}

int main() { //DON'T TOUCH

  //Run the pre-autonomous function. 
  pre_auton();

  //Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  //Prevent main from exiting with an infinite loop.                        
  while (1) {
    vex::task::sleep(100); //Sleep the task for a short amount of time to prevent wasted resources.
  }

}