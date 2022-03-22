// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // drivetrain motors
  private TalonSRX driveRightTalon = new TalonSRX(1);
  private TalonSRX driveLeftTalon = new TalonSRX(2);
  private CANSparkMax driveRightSpark = new CANSparkMax(3, MotorType.kBrushed);
  private CANSparkMax driveLeftSpark = new CANSparkMax(4, MotorType.kBrushed);

  // cargo movement motors
  private CANSparkMax launcherSpark = new CANSparkMax(5, MotorType.kBrushed);
  private Spark triggerSpark = new Spark(0);
  private Spark indexerSpark = new Spark(1);
  private Spark intakeSpark = new Spark(2);

  private Joystick joyDrive = new Joystick(0);
  private Joystick joyLauncher = new Joystick(1);
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // climber motor
  private Spark climberSpark = new Spark(3);

  // unit conversion - needed to convert # of ticks in a full rotation to # of feet
  private final double kDriveTick2Feet = 1.0/4096*6*Math.PI/12;

  // Gyro 180 turn variables
  double leftSlow = 0.24;
  double rightSlow = -0.24;
  double rotateSpeed = 0.35;
  double rotateSpeedSlow = 0.25; 

  Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() { 

    // invert settings
    driveLeftTalon.setInverted(true);

    driveRightSpark.setInverted(true);
    driveLeftSpark.setInverted(true);

    // init encoders
    driveLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); 
    driveRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //The "10" is the timeoutMS - set as non-zero so robot will retry if sending command fails

    driveLeftTalon.setSensorPhase(false);
    driveRightTalon.setSensorPhase(true);

    // reset encoders to zero
    driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
    driveRightTalon.setSelectedSensorPosition(0, 0, 10);

    // camera on Dashboard
    CameraServer.startAutomaticCapture();

  }

  @Override
  public void robotPeriodic() {
    // putting encooder values up on SmartDashboard
    SmartDashboard.putNumber("Left Motor Encoder Value:", driveLeftTalon.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Motor Encoder Value:", driveRightTalon.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("angle 1", gyro.getAngle());
    SmartDashboard.putNumber("angle 2", Math.abs(gyro.getAngle() % 360));

  }

  @Override
  public void autonomousInit() {
    // reset encoders to zero again - just in case!
    driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
    driveRightTalon.setSelectedSensorPosition(0, 0, 10);

    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    startTime = Timer.getFPGATimestamp();

  }

  // init variables for PID

  final double kP = 0.11;  // NEED TO ADJUST! DO THIS ONE FIRST!
  final double kI = 0.2;  // NEED TO ADJUST!
  final double kD = 0;  // NEED TO ADJUST!
  final double iLimit = 1;  // distance from set point when PID starts - NEED TO ADJUST?

  double setpoint = 8.25;
  double errorSum = 0;
  double lastTimestamp = 0;
  double startTime = 0;
  double lastError = 0;
  boolean start180 = false;
  double setPoint180 = setpoint + 3.3;
  boolean movetoLaunchSetpoint = false;
  double launchSetpoint = 3;
  boolean encodersReset = false;
  double finalSetPoint = -3;
  boolean atLaunchSetpoint = false;
  boolean atFinalSetpoint = false;

  @Override
  public void autonomousPeriodic() {

    // get sensor position
    double leftPosition = driveLeftTalon.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = driveRightTalon.getSelectedSensorPosition() * kDriveTick2Feet * -1; //rightPosition is a negative number!
    double sensorPosition = (leftPosition + rightPosition)/2;

    SmartDashboard.putNumber("leftPosition:", leftPosition);
    SmartDashboard.putNumber("rightPosition:", rightPosition);
    SmartDashboard.putNumber("distance from start:", sensorPosition);
    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("startTime", startTime);
    SmartDashboard.putNumber("current time", Timer.getFPGATimestamp());
    SmartDashboard.putBoolean("start180", start180);
    SmartDashboard.putBoolean("movetoLaunchSetpoint", movetoLaunchSetpoint);

    indexerSpark.set(-1);

      // PART 1: MOVE FORWARD AND PICK UP CARGO
      // First second of autonomous - drive forward.
    if (!start180){
      if (Timer.getFPGATimestamp()-startTime<1){
        driveLeftTalon.set(ControlMode.PercentOutput, 0.4);
        driveRightTalon.set(ControlMode.PercentOutput, -0.4);
        driveLeftSpark.set(0.4);
        driveRightSpark.set(-0.4);
      // Next 0.5 seconds of autonomous - stop drive motors, intake will fall into place
      } else if (Timer.getFPGATimestamp()-startTime<1.5){
        driveLeftTalon.set(ControlMode.PercentOutput, -0.5);
        driveRightTalon.set(ControlMode.PercentOutput, 0.5);
        driveLeftSpark.set(0);
        driveRightSpark.set(0);
      // Rest of autonomous drive up until pick up position (setpoint)
      } else {
        // calculations (error = distance from setpoint)
        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        SmartDashboard.putNumber("error", error);

        if (Math.abs(error) < iLimit) {
          errorSum += error * dt;
          // run intake when within 1 foot of setpoint
          intakeSpark.set(-0.7);    
        }

        double errorRate = (error - lastError) / dt;

        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;


        // after pickup setpoint is reached, turn 180
        if (error < 0.01 && start180 == false) {
          start180 = true;
        }
      
        // output to motors
        driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
        driveRightTalon.set(ControlMode.PercentOutput, -outputSpeed);
        driveLeftSpark.set(outputSpeed);
        driveRightSpark.set(-outputSpeed);
        
        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
      } //end of else statement - end of autonomous drive to setpoint
    }

    //PART 2: 180 TURN

    if (start180 && !movetoLaunchSetpoint) {
      /* attempt #1: using gyro (not tested! maybe later!)
      if (Math.abs(gyro.getAngle()) <= 3) {
        driveLeftSpark.set(leftSlow - (gyro.getAngle()) / 15);
        driveLeftTalon.set(ControlMode.PercentOutput, leftSlow - (gyro.getAngle()) / 15);
        driveRightSpark.set(rightSlow - (gyro.getAngle()) / 15);
        driveRightTalon.set(ControlMode.PercentOutput, rightSlow - (gyro.getAngle()) / 15);
      } else if (Math.abs(gyro.getAngle()) < 10) {
        if (gyro.getAngle() > 0) {
          driveLeftSpark.set(leftSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,leftSlow);
          driveRightSpark.set(1.1 * rightSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,1.1 * rightSlow);
        } else if (gyro.getAngle() < 0) {
          driveLeftSpark.set(1.1 * leftSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,1.1 * leftSlow);
          driveRightSpark.set(rightSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,rightSlow);
        }
      } else
        if (gyro.getAngle() > 0) {
          while (gyro.getAngle() > 10 && isAutonomous()) {
            driveLeftSpark.set(-rotateSpeed);
            driveLeftTalon.set(ControlMode.PercentOutput,-rotateSpeed);
            driveRightSpark.set(-rotateSpeed);
            driveRightTalon.set(ControlMode.PercentOutput,-rotateSpeed);
          }
          while (gyro.getAngle() > 0 && isAutonomous()) {
          driveLeftSpark.set(-rotateSpeedSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,-rotateSpeedSlow);
          driveRightSpark.set(-rotateSpeedSlow);
          driveRightTalon.set(ControlMode.PercentOutput,-rotateSpeedSlow);
          }
          while (gyro.getAngle() < 0 && isAutonomous()) {
          driveLeftSpark.set(rotateSpeedSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,rotateSpeedSlow);
          driveRightSpark.set(rotateSpeedSlow);
          driveRightTalon.set(ControlMode.PercentOutput,rotateSpeedSlow);
          }
        } else {
          while (gyro.getAngle() < -10 && isAutonomous()) {
          driveLeftSpark.set(rotateSpeed);
          driveLeftTalon.set(ControlMode.PercentOutput,rotateSpeed);
          driveRightSpark.set(rotateSpeed);
          driveRightTalon.set(ControlMode.PercentOutput,rotateSpeed);
          }
          while (gyro.getAngle() < 0 && isAutonomous()) {
          driveLeftSpark.set(rotateSpeedSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,rotateSpeedSlow);
          driveRightSpark.set(rotateSpeedSlow);
          driveRightTalon.set(ControlMode.PercentOutput,rotateSpeedSlow);
          }
          while (gyro.getAngle() > 0 && isAutonomous()) {
          driveLeftSpark.set(-rotateSpeedSlow);
          driveLeftTalon.set(ControlMode.PercentOutput,-rotateSpeedSlow);
          driveRightSpark.set(-rotateSpeedSlow);
          driveRightTalon.set(ControlMode.PercentOutput,-rotateSpeedSlow);
          }
        } 
      }*/
      
      // Attempt #2: using encoders only
      double error = setPoint180 - leftPosition;
      SmartDashboard.putNumber("error", error);

      double outputSpeed = kP * error;

      // move cargo into position
      indexerSpark.set(-1);
      launcherSpark.set(-0.7); 

      // output to motors
      driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveRightTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveLeftSpark.set(outputSpeed);
      driveRightSpark.set(outputSpeed);

      // update last- variables
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;    
      SmartDashboard.putNumber("outputSpeed", outputSpeed);
     
      if (outputSpeed < 0.06) {
        movetoLaunchSetpoint = true;
        startTime = Timer.getFPGATimestamp();
       }
    } // end of 180 turn

    // PART 3: MOVE TO LAUNCH POSITION
    if (movetoLaunchSetpoint && !atLaunchSetpoint) {
 
      // keep moving cargo into position
      indexerSpark.set(-1);
      launcherSpark.set(-0.7); 

      /*
      
      // calculations (error = distance from setpoint)
      double error = launchSetpoint - sensorPosition;
      SmartDashboard.putNumber("error", error);

      double outputSpeed = kP * error;

      // output to motors
      driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveRightTalon.set(ControlMode.PercentOutput, -outputSpeed);
      driveLeftSpark.set(outputSpeed);
      driveRightSpark.set(-outputSpeed);
      
      // update last- variables
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;
      */

      double error = 0; // remove this when above code is used!

      if (error < 0.01) {
        atLaunchSetpoint = true;
      }
      triggerSpark.set(-1);
 
    }

    // PART 4: LAUNCH!

    if (atLaunchSetpoint){

      triggerSpark.set(-1);
      double error = 0;

      if (Timer.getFPGATimestamp()-startTime>1 && Timer.getFPGATimestamp()-startTime<1.25){
        // output to motors
        driveLeftTalon.set(ControlMode.PercentOutput, 0.5);
        driveRightTalon.set(ControlMode.PercentOutput, -0.5);
        driveLeftSpark.set(0.5);
        driveRightSpark.set(-0.5);
      } else if (Timer.getFPGATimestamp()-startTime>1.25 && Timer.getFPGATimestamp()-startTime<1.5){   
        // output to motors
        driveLeftTalon.set(ControlMode.PercentOutput, -0.5);
        driveRightTalon.set(ControlMode.PercentOutput, 0.5);
        driveLeftSpark.set(-0.5);
        driveRightSpark.set(0.5);        
      } else if (Timer.getFPGATimestamp()-startTime>1.5 && Timer.getFPGATimestamp()-startTime<4.5) {
        // output to motors
        driveLeftTalon.set(ControlMode.PercentOutput, 0);
        driveRightTalon.set(ControlMode.PercentOutput, 0);
        driveLeftSpark.set(0);
        driveRightSpark.set(0);
        lastTimestamp = 0;
      } else { // move completely out of Tarmac area      

      // calculations (error = distance from setpoint)
      error = finalSetPoint - sensorPosition;
      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      SmartDashboard.putNumber("error", error);

      if (Math.abs(error) < iLimit) {
        errorSum += error * dt; 
      }

      double errorRate = (error - lastError) / dt;

      double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

      /* CODE ABOVE NEEDS TO BE TESTED FIRST!!!
      // output to motors
      driveLeftTalon.set(ControlMode.PercentOutput, -outputSpeed);
      driveRightTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveLeftSpark.set(-outputSpeed);
      driveRightSpark.set(outputSpeed);
      
      */

      // update last- variables
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;
               
      }

    }


  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    driveLeftTalon.setNeutralMode(NeutralMode.Coast);
    driveRightTalon.setNeutralMode(NeutralMode.Coast);

    double speed = -joyDrive.getRawAxis(1) * 0.6;
    double turn = joyDrive.getRawAxis(4) * 0.3;

    // deadband - if joystick is resting SLIGHTLY off centre, will ignore value
    if (Math.abs(speed) < 0.05) {
      speed = 0;
    }
    if (Math.abs(turn) < 0.05) {
      turn = 0;
    }

    // fixing inverted reverse

    double left = 0;
    double right = 0;

    if (speed <= -0.01) {
      left = speed - turn;
      right = speed + turn;  
    } else {
      left = speed + turn;
      right = speed - turn;
  }

    driveLeftTalon.set(ControlMode.PercentOutput,left);
    driveRightTalon.set(ControlMode.PercentOutput,-right);
    driveLeftSpark.set(left);
    driveRightSpark.set(-right);

  // setting button actions

  double launcherPower = 0;
  double indexerPower = 0;
  double intakePower = 0;
  double climberPower = 0;

//  if (joyDrive.getRawButton(1)==true){
  if (joyLauncher.getRawAxis(3)>0.1){
    launcherPower = -0.7;
    indexerPower = -1;
  }
  
  double triggerPower = 0;
  // if (joyDrive.getRawButton(2)==true){
  
  if (joyLauncher.getRawAxis(2)>0.1){
    triggerPower = joyLauncher.getRawAxis(2)*(-1);
  //  triggerPower = 0.4;
  }

  triggerSpark.set(triggerPower);

  if (joyLauncher.getRawButton(1)==true){
    intakePower = -0.7;
    indexerPower = -1;
  }

  if (joyLauncher.getRawButton(3)==true){
    climberPower = 0.5;
  }

  if (joyLauncher.getRawButton(4)==true){
    climberPower = -0.5;
  }

  intakeSpark.set(intakePower);    
  launcherSpark.set(launcherPower);
  indexerSpark.set(indexerPower);
  climberSpark.set(climberPower);

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}