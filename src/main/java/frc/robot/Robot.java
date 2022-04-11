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
//import edu.wpi.first.math.controller.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
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

  // unit conversion - needed to convert # of ticks in a full rotation to # of
  // feet
  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;

  // Gyro 180 turn variables
  double leftSlow = 0.24;
  double rightSlow = -0.24;
  double rotateSpeed = 0.35;
  double rotateSpeedSlow = 0.25;

  Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
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
    driveRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); 
    driveLeftTalon.setSensorPhase(false);
    driveRightTalon.setSensorPhase(true);

    // reset encoders to zero
    driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
    driveRightTalon.setSelectedSensorPosition(0, 0, 10);

    // camera on Dashboard
    CameraServer.startAutomaticCapture();

    // initialize gyro
    gyro.calibrate();
  }

  @Override
  public void robotPeriodic() {
    // putting encooder values up on SmartDashboard
    SmartDashboard.putNumber("Left Motor Encoder Value:", driveLeftTalon.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Motor Encoder Value:", driveRightTalon.getSelectedSensorPosition() * kDriveTick2Feet);
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
  final double kP = 0.2;
  final double kI = 0;
  final double kD = 0;
  final double launchkP = 0.2;
  final double launchkI = 0;
  final double launchkD = 0;
  final double iLimit = 1; // distance from setpoint when PID starts (# of feet)
  final double rotatekP = 0.00408;
  final double rotatekI = 0.00544;
  final double rotatekD = 0.000765;
  final double iLimit180 = 30; // # of degrees from setpoint when PID starts

  double errorSum = 0;
  double lastTimestamp = 0;
  double startTime = 0;
  double lastError = 0;

  // define set points for autonomous
  // double setpoint = -5; //ONLY USED FOR AUTONOMOUS A & C: BACK UP AND SHOOT OR BACK UP ONLY
  double setpoint = 6.5; // ONLY USED FOR AUTONOMOUS B: FULL AUTO WITH 180 TURN AND FIRE
  double setPointTurn = 180; // for gyro turn
  // double setPointTurn = 3.35; // for encoder (turn value 3.35 = 180 degree turn on linoleum)
  double launchSetpoint = 2.5;
  double finalSetPoint = -8;

  // define boolean for each stage of autonomous - will only move to next stage if true
  boolean start180 = false;
  boolean movetoLaunchSetpoint = false;
  boolean atLaunchSetpoint = false;
  boolean atFinalSetpoint = false;
  boolean leaveTarmac = false;

  boolean encodersReset = false;

  @Override
  /*
   * public void autonomousPeriodic() { // AUTONOMOUS A - simple back up and fire
   * 
   * // get sensor position
   * double leftPosition = driveLeftTalon.getSelectedSensorPosition() *
   * kDriveTick2Feet;
   * double rightPosition = driveRightTalon.getSelectedSensorPosition() *
   * kDriveTick2Feet * -1; //rightPosition is a negative number!
   * double sensorPosition = (leftPosition + rightPosition)/2;
   * 
   * SmartDashboard.putNumber("leftPosition:", leftPosition);
   * SmartDashboard.putNumber("rightPosition:", rightPosition);
   * SmartDashboard.putNumber("distance from start:", sensorPosition);
   * SmartDashboard.putNumber("startTime", startTime);
   * SmartDashboard.putNumber("current time", Timer.getFPGATimestamp());
   * 
   * indexerSpark.set(-1);
   * launcherSpark.set(-0.6);
   * 
   * // calculations (error = distance from setpoint)
   * double error = setpoint - sensorPosition;
   * SmartDashboard.putNumber("error", error);
   * 
   * double dt = Timer.getFPGATimestamp() - lastTimestamp;
   * 
   * if (Math.abs(error) < iLimit) {
   * errorSum += error * dt;
   * // run intake when within 1 foot of setpoint
   * //intakeSpark.set(-0.6);
   * }
   * 
   * double errorRate = (error - lastError) / dt;
   * 
   * double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
   * SmartDashboard.putNumber("auto outputSpeed", outputSpeed);
   * 
   * // output to motors
   * driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
   * driveRightTalon.set(ControlMode.PercentOutput, -outputSpeed);
   * driveLeftSpark.set(outputSpeed);
   * driveRightSpark.set(-outputSpeed);
   * 
   * // update last- variables
   * lastTimestamp = Timer.getFPGATimestamp();
   * lastError = error;
   * 
   * if (Math.abs(error) < 0.03) {
   * triggerSpark.set(-1);
   * }
   * }
   */

  /*
   * public void autonomousPeriodic() { // AUTONOMOUS C - simple back up, NO fire!
   * 
   * // get sensor position
   * double leftPosition = driveLeftTalon.getSelectedSensorPosition() * kDriveTick2Feet;
   * double rightPosition = driveRightTalon.getSelectedSensorPosition() * kDriveTick2Feet * -1; //rightPosition is a negative number!
   * double sensorPosition = (leftPosition + rightPosition)/2;
   * 
   * SmartDashboard.putNumber("leftPosition:", leftPosition);
   * SmartDashboard.putNumber("rightPosition:", rightPosition);
   * SmartDashboard.putNumber("distance from start:", sensorPosition);
   * SmartDashboard.putNumber("startTime", startTime);
   * SmartDashboard.putNumber("current time", Timer.getFPGATimestamp());
   * 
   * // calculations (error = distance from setpoint)
   * double error = setpoint - sensorPosition;
   * SmartDashboard.putNumber("error", error);
   * 
   * double dt = Timer.getFPGATimestamp() - lastTimestamp;
   * 
   * if (Math.abs(error) < iLimit) {
   * errorSum += error * dt;
   * }
   * 
   * double errorRate = (error - lastError) / dt;
   * 
   * double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
   * SmartDashboard.putNumber("auto outputSpeed", outputSpeed);
   * 
   * // output to motors
   * driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
   * driveRightTalon.set(ControlMode.PercentOutput, -outputSpeed);
   * driveLeftSpark.set(outputSpeed);
   * driveRightSpark.set(-outputSpeed);
   * 
   * // update last- variables
   * lastTimestamp = Timer.getFPGATimestamp();
   * lastError = error;
   * 
   * }
   * }
   */
  public void autonomousPeriodic() { // AUTONOMOUS B - FULL AUTONOMOUS USING ENCODERS OR GYRO

    // get sensor position
    double leftPosition = driveLeftTalon.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = driveRightTalon.getSelectedSensorPosition() * kDriveTick2Feet * -1; // rightPosition is a negative number!
    double sensorPosition = (leftPosition + rightPosition) / 2;

    SmartDashboard.putNumber("leftPosition:", leftPosition);
    SmartDashboard.putNumber("rightPosition:", rightPosition);
    SmartDashboard.putNumber("distance from start:", sensorPosition);
    SmartDashboard.putNumber("startTime", startTime);
    SmartDashboard.putNumber("current time", Timer.getFPGATimestamp());
    SmartDashboard.putBoolean("start180", start180);
    SmartDashboard.putBoolean("movetoLaunchSetpoint", movetoLaunchSetpoint);
    SmartDashboard.putNumber("sensorPosition", sensorPosition);
    SmartDashboard.putBoolean("atLaunchSetpoint", atLaunchSetpoint);
    SmartDashboard.putBoolean("leaveTarmac", leaveTarmac);

    indexerSpark.set(-1);

    // PART 1: MOVE FORWARD AND PICK UP CARGO
    // First second of autonomous - drive forward.
    if (!start180) {
      if (Timer.getFPGATimestamp() - startTime < 1) {
        driveLeftTalon.set(ControlMode.PercentOutput, 0.4);
        driveRightTalon.set(ControlMode.PercentOutput, -0.4); 
        driveLeftSpark.set(0.4);
        driveRightSpark.set(-0.4);
        // Next 0.5 seconds of autonomous - stop drive motors, intake will fall into
        // place
      } else if (Timer.getFPGATimestamp() - startTime < 1.5) {
        driveLeftTalon.set(ControlMode.PercentOutput, -0.5);
        driveRightTalon.set(ControlMode.PercentOutput, 0.5);
        driveLeftSpark.set(0);
        driveRightSpark.set(0);
        intakeSpark.set(-0.6);
        // Rest of autonomous drive up until pick up position (setpoint)
      } else {
        // calculations (error = distance from setpoint)
        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        intakeSpark.set(-0.6);


        if (Math.abs(error) < iLimit) {
          errorSum += error * dt;
          // start running intake when within 1 foot of setpoint
        }

        double errorRate = (error - lastError) / dt;
        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

        // after pickup setpoint is reached, go to part 2: 180 turn
        if (error < 0.01 && start180 == false) {
          start180 = true;
          // reset encoders and gyro to zero
          driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
          driveRightTalon.setSelectedSensorPosition(0, 0, 10);
          gyro.reset();
        }

        // output to motors
        driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
        driveRightTalon.set(ControlMode.PercentOutput, -outputSpeed);
        driveLeftSpark.set(outputSpeed);
        driveRightSpark.set(-outputSpeed);

        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
      } // end of else statement - end of autonomous drive to setpoint

      // PART 2: 180 TURN
    } else if (start180 && !movetoLaunchSetpoint) {
      // attempt #2: using gyro - NO PID!
  /*    double outputSpeed = 0;
      double angle = Math.abs(gyro.getAngle());

      double error = setPointTurn - angle;

      if ((angle < setPointTurn) && (error > 90)) {
        outputSpeed = 0.3;
      } else if ((angle < setPointTurn) && (error > 1)){
        outputSpeed = 0.125;
      } else {
        outputSpeed = -0.01;  // hard stop!
      }

      // output to motors
      driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveRightTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveLeftSpark.set(outputSpeed);
      driveRightSpark.set(outputSpeed);

      if (Math.abs(error) < 3) {
        movetoLaunchSetpoint = true;
        // reset startTime for next part
        startTime = Timer.getFPGATimestamp();
        driveLeftTalon.set(ControlMode.PercentOutput, 0);
        driveRightTalon.set(ControlMode.PercentOutput, 0);
        driveLeftSpark.set(0);
        driveRightSpark.set(0);
        // reset encoders to zero to move to launch position
        driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
        driveRightTalon.setSelectedSensorPosition(0, 0, 10);
      }
      SmartDashboard.putNumber("180 error", error);
      SmartDashboard.putNumber("calculated angle", angle);
      SmartDashboard.putNumber("angle", gyro.getAngle());
      SmartDashboard.putNumber("180 outputSpeed", outputSpeed);
*/     
        // attempt #3: using gyro + PID (WORKS!!!)
        double angle = Math.abs(gyro.getAngle());

        double error = setPointTurn - angle;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
    
        if (Math.abs(error) < iLimit180) {
          errorSum += error * dt;
        }

        if (Math.abs(error) < 2) {
          movetoLaunchSetpoint = true;
          // reset encoders to zero
          driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
          driveRightTalon.setSelectedSensorPosition(0, 0, 10);
        }

        double errorRate = (error - lastError) /dt;
        double outputSpeed = rotatekP * error + rotatekI * errorSum + rotatekD * errorRate;
    
        // output to motors
        driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
        driveRightTalon.set(ControlMode.PercentOutput, outputSpeed);
        driveLeftSpark.set(outputSpeed);
        driveRightSpark.set(outputSpeed);
    
        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
        
      /*
       * // Attempt #1: using encoders only - WORKS, BUT NOT WELL!!!
       * 
       * // move cargo into position - always running from now on!
       * indexerSpark.set(-1);
       * launcherSpark.set(-0.55);
       * 
       * double error = setPointTurn - leftPosition;
       * double dt = Timer.getFPGATimestamp() - lastTimestamp;
       * SmartDashboard.putNumber("setPointTurn", setPointTurn);
       * 
       * 
       * if (Math.abs(error) < (iLimit180)) {
       * errorSum += error * dt;
       * }
       * 
       * double outputSpeed = kP * error;
       * 
       * // output to motors
       * driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
       * driveRightTalon.set(ControlMode.PercentOutput, outputSpeed);
       * driveLeftSpark.set(outputSpeed);
       * driveRightSpark.set(outputSpeed);
       * 
       * // update last- variables
       * lastTimestamp = Timer.getFPGATimestamp();
       * lastError = error;
       * SmartDashboard.putNumber("outputSpeed", outputSpeed);
       * SmartDashboard.putNumber("180 error", error);
       * 
       * if (Math.abs(error) < (0.5)) {
       * movetoLaunchSetpoint = true;
       * // reset startTime for next part
       * startTime = Timer.getFPGATimestamp();
       * // reset encoders to zero to move to launch position
       * driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
       * driveRightTalon.setSelectedSensorPosition(0, 0, 10);
       * }
       */

      // PART 3: MOVE TO LAUNCH POSITION
    } else if (movetoLaunchSetpoint && !atLaunchSetpoint) {
      // keep moving cargo into position - still always on!
      indexerSpark.set(-1);
      launcherSpark.set(-0.55);

      // calculations (error = distance from launchSetpoint)
      double error = launchSetpoint - sensorPosition;
      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      SmartDashboard.putNumber("error Launch", error);

      if (Math.abs(error) < iLimit) {
        errorSum += error * dt;
      }

      double errorRate = (error - lastError) / dt;

      double outputSpeed = launchkP * error + launchkI * errorSum + launchkD * errorRate;

      // output to motors
      driveLeftTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveRightTalon.set(ControlMode.PercentOutput, -outputSpeed);
      driveLeftSpark.set(outputSpeed);
      driveRightSpark.set(-outputSpeed);

      // update last- variables
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;

      if (Math.abs(error) < 0.2) {
        atLaunchSetpoint = true;
        driveLeftTalon.set(ControlMode.PercentOutput, 0);
        driveRightTalon.set(ControlMode.PercentOutput, 0);
        driveLeftSpark.set(0);
        driveRightSpark.set(0);
        // reset encoders to zero to back out of Tarmac area
        driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
        driveRightTalon.setSelectedSensorPosition(0, 0, 10);
        // reset startTime
        startTime = Timer.getFPGATimestamp();
      }
      //triggerSpark.set(-1);

      // PART 4: LAUNCH!

    } else if (atLaunchSetpoint && !leaveTarmac) {

      // double error = 0; DELETE IF NOT NEEDED!

      if (Timer.getFPGATimestamp() - startTime < 2) {
        // wait for 1 second to launch the first cargo
        driveLeftTalon.set(ControlMode.PercentOutput, 0);
        driveRightTalon.set(ControlMode.PercentOutput, 0);
        driveLeftSpark.set(0);
        driveRightSpark.set(0);
      } else if (Timer.getFPGATimestamp()-startTime < 5) {
        triggerSpark.set(-1);
      /* NOT NEEDED ANYMORE???
      } else if (Timer.getFPGATimestamp() - startTime > 1 &&
          Timer.getFPGATimestamp() - startTime < 1.25) {
        // jiggle forward and back to get second cargo into place
        driveLeftTalon.set(ControlMode.PercentOutput, -0.5);
        driveRightTalon.set(ControlMode.PercentOutput, 0.5);
        driveLeftSpark.set(-0.5);
        driveRightSpark.set(0.5);
      } else if (Timer.getFPGATimestamp() - startTime > 1.25 &&
          Timer.getFPGATimestamp() - startTime < 1.5) {
        driveLeftTalon.set(ControlMode.PercentOutput, 0.5);
        driveRightTalon.set(ControlMode.PercentOutput, -0.5);
        driveLeftSpark.set(0.5);
        driveRightSpark.set(-0.5);
      } else if (Timer.getFPGATimestamp() - startTime > 1.5 &&
          Timer.getFPGATimestamp() - startTime < 4.5) {
        // wait 3 seconds to fire 2nd cargo
        driveLeftTalon.set(ControlMode.PercentOutput, 0);
        driveRightTalon.set(ControlMode.PercentOutput, 0);
        driveLeftSpark.set(0);
        driveRightSpark.set(0);
        // reset encoders to zero to back out of Tarmac area
        driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
        driveRightTalon.setSelectedSensorPosition(0, 0, 10);   */
      } else if (Timer.getFPGATimestamp() - startTime > 5) {
        leaveTarmac = true;
        lastTimestamp = 0;
        // reset encoders to zero to back out of Tarmac area
        driveLeftTalon.setSelectedSensorPosition(0, 0, 10);
        driveRightTalon.setSelectedSensorPosition(0, 0, 10);
      }

      // PART 5: BACK OUT OF TARMAC
    } else if (leaveTarmac) {
      // move completely out of Tarmac area - back up 5 feet

      launcherSpark.set(0);
      triggerSpark.set(0);
      indexerSpark.set(0);
      intakeSpark.set(0);

      // calculations (error = distance from setpoint)
      double error = finalSetPoint - sensorPosition;
      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      SmartDashboard.putNumber("Tarmac error", error);

      if (Math.abs(error) < iLimit) {
        errorSum += error * dt;
      }

      double errorRate = (error - lastError) / dt;

      double outputSpeed = -1 * (kP * error + kI * errorSum + kD * errorRate); // -1 need to move in reverse!

      // output to motors
      driveLeftTalon.set(ControlMode.PercentOutput, -outputSpeed);
      driveRightTalon.set(ControlMode.PercentOutput, outputSpeed);
      driveLeftSpark.set(-outputSpeed);
      driveRightSpark.set(outputSpeed);

      // update last- variables
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;
    }
  }

  @Override
  public void teleopInit() {
  }

  boolean reverse = false;

  @Override
  public void teleopPeriodic() {

    if (joyDrive.getRawAxis(3) > 0.5) { // if right trigger then direction reversed - backward is now forward
      reverse = true;
    } else {
      reverse = false;
    }

    driveLeftTalon.setNeutralMode(NeutralMode.Coast);
    driveRightTalon.setNeutralMode(NeutralMode.Coast);

    double speed = -joyDrive.getRawAxis(1) * 0.8;
    double turn = joyDrive.getRawAxis(4) * 0.4;

    // makes turn less touchy for positioning when starting climb
    if (joyDrive.getRawAxis(2) > 0.1) {
      turn = joyDrive.getRawAxis(4) * 0.15;
    }

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

    if (reverse) {
      if (speed <= -0.01) {
        left = speed + turn;
        right = speed - turn;
      } else {
        left = speed - turn;
        right = speed + turn;
      }
      driveLeftTalon.set(ControlMode.PercentOutput, -left);
      driveRightTalon.set(ControlMode.PercentOutput, right);
      driveLeftSpark.set(-left);
      driveRightSpark.set(right);
    } else {
      if (speed <= -0.01) {
        left = speed - turn;
        right = speed + turn;
      } else {
        left = speed + turn;
        right = speed - turn;
      }
      driveLeftTalon.set(ControlMode.PercentOutput, left);
      driveRightTalon.set(ControlMode.PercentOutput, -right);
      driveLeftSpark.set(left);
      driveRightSpark.set(-right);
    }

    // setting button actions

    double launcherPower = 0;
    double indexerPower = 0;
    double intakePower = 0;
    double climberPower = 0;

    if (joyLauncher.getRawAxis(3) > 0.1) {
      launcherPower = -0.55;
      indexerPower = -1;
    }

    double triggerPower = 0;

    if (joyLauncher.getRawAxis(2) > 0.1) {
      triggerPower = joyLauncher.getRawAxis(2) * (-1);
    }

    triggerSpark.set(triggerPower);

    if (joyLauncher.getRawButton(1) == true) {
      intakePower = -0.6;
      indexerPower = -1;
    }

    if (joyLauncher.getRawButton(3) == true) {
      climberPower = 1;
    }

    if (joyLauncher.getRawButton(4) == true) {
      climberPower = -1;
    }

    if (joyLauncher.getRawButton(2) == true) { // reverse intake to unstick ball!
      intakePower = 1;
    }

    intakeSpark.set(intakePower);
    launcherSpark.set(launcherPower);
    indexerSpark.set(indexerPower);
    climberSpark.set(climberPower);

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}