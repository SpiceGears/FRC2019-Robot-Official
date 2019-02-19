/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Climbing;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Climbing.Mode;

/**
 * Add your docs here.
 */
public class OpenClimbDriveTrain extends InstantCommand {
  /**
   * Add your docs here.
   */
  public OpenClimbDriveTrain() {
    super();
    requires(Robot.climbing);
    requires(Robot.intakeLifting);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.climbing.setMode(Mode.OPEN);
    Robot.intakeLifting.setIsCheckEncoder(false);
  }

}
