// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final Spark intakeMotor = new Spark(5);   // NEED TO GET ID FOR THIS!!!!!!!!

  public IntakeSubsystem() {}

  public void setPosition(boolean open) {
    if (open) {
      intakeMotor.set(-1);
    } else {
      intakeMotor.set(1);
    }
  }

}
