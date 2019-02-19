/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Drive;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.SpeedPID;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveToTarget extends Command {
  double x;
  double y;
  double angle;
  Trajectory trajectory;
  Trajectory left;
  Trajectory right;
  long oldTime = 0;
  int index = 0;
  double startTime;
  SpeedPID speedPIDr;
  SpeedPID speedPIDl;
  int oldIndex = 0;

  
  public DriveToTarget(double x, double y, double angle) {
    requires(Robot.driveTrain);
    this.x = x;
    this.y = y;
    this.angle = angle;
    speedPIDr = new SpeedPID(Constants.SpeedDrivePIDkP, Constants.SpeedDrivePIDkI, Constants.SpeedDrivePIDkD, Constants.SpeedDrivePIDkF);
    speedPIDl = new SpeedPID(Constants.SpeedDrivePIDkP, Constants.SpeedDrivePIDkI, Constants.SpeedDrivePIDkD, Constants.SpeedDrivePIDkF);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    
    index = 0;
    oldIndex = 0;
    oldTime = 0;
    SmartDashboard.putNumber("debugowanie", 0);
      Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 0.05, 2, 1.5, 60.0);
      Waypoint[] points = new Waypoint[] {
        new Waypoint(0.1, 0, 0),
        new Waypoint(x, y, Pathfinder.d2r(angle))
      };
      SmartDashboard.putNumber("debugowanie", 1);
      trajectory = Pathfinder.generate(points, config); 
    // Create the Trajectory Configuration
    //
    // Arguments:
    // Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
    // Sample Count:        SAMPLES_HIGH (100 000)
    //                      SAMPLES_LOW  (10 000)
    //                      SAMPLES_FAST (1 000)
    // Time Step:           0.05 Seconds
    // Max Velocity:        1.7 m/s
    // Max Acceleration:    2.0 m/s/s
    // Max Jerk:            60.0 m/s/s/s
    // Generate the trajectory
    TankModifier modifier = new TankModifier(trajectory).modify(0.595);
  
    left = modifier.getLeftTrajectory();
    right = modifier.getRightTrajectory();
  
    startTime = Timer.getFPGATimestamp() * 1000;

    Robot.driveTrain.resetGyro();
  }
//index = ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 20));
  // Called repeatedly when this Command is scheduled to run

  double leftSet = 0;
  double rightSet = 0;
  @Override
  protected void execute() {

    index = ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 200));

        if(index > oldIndex){
            oldIndex = index;

          leftSet = speedPIDr.calculate(left.segments[index].velocity, Robot.driveTrain.getLeftActualSpeed());
          rightSet = speedPIDl.calculate(right.segments[index].velocity, Robot.driveTrain.getRightActualSpeed());

        } else {
            Timer.delay(0.01);
        }
        SmartDashboard.putNumber("debugowanie", 13);
      double gyro_heading = Robot.driveTrain.getHeading();// Assuming gyro angle is given in degrees
      double desired_heading = Pathfinder.r2d(left.segments[index].heading);
      double angle_difference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);// Make sure to bound this from -180 to 180, otherwise you will get super large values
      //System.out.printf("Desired Heading = %03.2f ; Gyro Heading = %03.2f ; Angle Difference = %03.2f ; Turn Sensitivity = %.4f \n", desired_heading, gyro_heading, angle_difference, turnSensitivity);
      double turn = Constants.DRIVERotacion_kP * angle_difference *(-1.0 / 80.0);
      
      SmartDashboard.putNumber("Angle", Robot.driveTrain.getHeading());
      SmartDashboard.putNumber("leftset", leftSet);
      SmartDashboard.putNumber("rightset", -rightSet);

      // SmartDashboard.putNumber("AngleDifference", angle_difference);
      // SmartDashboard.putNumber("Turn", turn);
      Robot.driveTrain.driveSet(leftSet+turn, -rightSet+turn);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    SmartDashboard.putNumber("index", index);
    SmartDashboard.putNumber("segment", left.segments.length);
    return index + 1 >= left.segments.length || index + 1 >= right.segments.length || Robot.oi.getDriverJoystick().getRawButton(1);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stopDrive();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
