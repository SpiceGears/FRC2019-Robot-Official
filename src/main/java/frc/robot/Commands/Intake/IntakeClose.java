/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class IntakeClose extends Command {
  /**
   * Add your docs here.
   */
  public IntakeClose() {
    super();
    requires(Robot.hatchIntake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.hatchIntake.hatchSolenoid(false);
    System.out.print("chatch close!!!!");
  }

  @Override
  protected boolean isFinished() {
      return false;
  }

  protected void end() {
    Robot.hatchIntake.hatchOFF();
  }

  protected void interrupted() {
    end();
  }
}
