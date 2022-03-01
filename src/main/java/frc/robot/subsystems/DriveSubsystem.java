// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final TalonSRX talonRight = new TalonSRX(1);
  private final TalonSRX talonLeft = new TalonSRX(2);
  
  private final CANSparkMax sparkRight = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax sparkLeft = new CANSparkMax(4, MotorType.kBrushed);

  // unit conversion - needed to convert # of ticks in a full rotation to # of feet
  private final double kDriveTick2Feet = 1.0/4096*6*Math.PI/12;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    // invert settings
    talonLeft.setInverted(true);

    sparkRight.setInverted(true);
    sparkLeft.setInverted(true);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Motor Encoder Value:", talonLeft.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Motor Encoder Value:", talonRight.getSelectedSensorPosition() * kDriveTick2Feet);

  }

  public void setMotors(double outputSpeed) {
    // output to motors
    talonLeft.set(ControlMode.PercentOutput, outputSpeed);
    talonRight.set(ControlMode.PercentOutput, -outputSpeed);
    sparkLeft.set(outputSpeed);
    sparkRight.set(-outputSpeed);
    
  }

}
