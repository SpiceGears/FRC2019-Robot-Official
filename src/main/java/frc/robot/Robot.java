/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.BallIntake;
import frc.robot.Subsystems.CatchTarget;
import frc.robot.Subsystems.Climbing;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.HatchIntake;
import frc.robot.Subsystems.IntakeLifting;

public class Robot extends TimedRobot {
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static OI oi;
  public static Elevator elevator;
  public static HatchIntake hatchIntake;
  public static DriveTrain driveTrain;
  public static IntakeLifting intakeLifting;
  public static Climbing climbing;
  public static CatchTarget catchTarget;
  public static BallIntake ballIntake;

  public int matchTime = 0;

  @Override
  public void robotInit() {

    catchTarget = new CatchTarget();
    elevator = new Elevator();
    hatchIntake = new HatchIntake();
    ballIntake = new BallIntake();
    driveTrain = new DriveTrain();
    intakeLifting = new IntakeLifting();
    climbing = new Climbing();
    oi = new OI();


    elevator.resetEncoder();
    intakeLifting.resetEncoder();
    climbing.resetEncoder();
    driveTrain.stopDrive();

  }

  public void robotPeriodic(){
        Robot.driveTrain.logs();
        Robot.elevator.logs();
        Robot.climbing.logs();
        Robot.intakeLifting.logs();
        Robot.catchTarget.logs();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void autonomousInit() {
    driveTrain.stopDrive();

  }

  @Override
  public void autonomousPeriodic() {
    Robot.catchTarget.detectionMode();
    Scheduler.getInstance().run();

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

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    driveTrain.stopDrive();

  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

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
    matchTime = (int) DriverStation.getInstance().getMatchNumber();
    SmartDashboard.putNumber("timer", matchTime); //For Johnny <3

    Robot.catchTarget.detectionMode();

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
