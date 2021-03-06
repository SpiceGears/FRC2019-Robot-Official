/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Subsystems.IntakeLifting.Mode;

/**
 * Add your docs here.
 */
public class Intake135UP extends InstantCommand {
  /**
   * Add your docs here.
   */
  public Intake135UP() {
    super();
    requires(Robot.intakeLifting);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.intakeLifting.setMode(Mode.DEG_135_UP);
  }

}
