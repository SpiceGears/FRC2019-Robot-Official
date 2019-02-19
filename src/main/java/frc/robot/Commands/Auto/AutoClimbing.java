/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Controllers.ClimbingController;
import frc.robot.Controllers.ElevatorController;
import frc.robot.motion.TrapezoidalMotionProfile;

public class AutoClimbing extends Command {

  TrapezoidalMotionProfile profileElevator, profileClimbing;
  double setPoint;
  double maxVelocity;
  double maxAcceleration;
  ElevatorController elevatorController;
  ClimbingController climbingController;

  public AutoClimbing(double setPoint, double maxVelocity, double maxAcceleration) {
    this.setPoint = setPoint;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    requires(Robot.climbing);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climbing.resetEncoder();
    profileElevator = new TrapezoidalMotionProfile(setPoint, maxVelocity, maxAcceleration);
    profileClimbing = new TrapezoidalMotionProfile(setPoint, maxVelocity*(41/25), maxAcceleration*(41/25));
    elevatorController = new ElevatorController(profileElevator);
    climbingController = new ClimbingController(profileClimbing);
  }

  @Override
  protected void execute() {
    elevatorController.update();
    climbingController.update();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.climbing.getDistance() <= Constants.allowedClimbingError && Robot.elevator.getDistance() <= Constants.allowedElevatorError;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stopElevator();
    Robot.climbing.stopClimb();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
