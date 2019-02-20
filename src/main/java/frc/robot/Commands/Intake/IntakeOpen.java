/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class IntakeOpen extends Command {
  /**
   * Add your docs here.
   */
  public IntakeOpen() {
    super();
    requires(Robot.hatchIntake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    SmartDashboard.putNumber("solenoid", 1);
    if(Robot.oi.getDriverJoystick().getRawButton(5)){
      Robot.hatchIntake.hatchSolenoid(true);
      SmartDashboard.putNumber("solenoid", 2);
      System.out.print("chatch open!!!!");
    }else if(Robot.oi.getDriverJoystick().getRawButton(6)){
      Robot.hatchIntake.hatchSolenoid(false);
    System.out.print("chatch close!!!!");
    }else{
      Robot.hatchIntake.hatchOFF();
    }
  }

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
