/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  @Override
  public void robotInit() {

    catchTarget = new CatchTarget();
    elevator = new Elevator();
    hatchIntake = new HatchIntake();
    driveTrain = new DriveTrain();
    intakeLifting = new IntakeLifting();
    climbing = new Climbing();
    oi = new OI();

    elevator.resetEncoder();
    intakeLifting.resetEncoder();
    climbing.resetEncoder();
    driveTrain.stopDrive();

  //   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  //   camera.setResolution(288, 216);
  //   camera.setFPS(20);

  // //   new Thread(() -> {
  // //     UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  // //     camera.setResolution(288, 216);
  // //     camera.setFPS(20);
      
  // //     CvSink cvSink = CameraServer.getInstance().getVideo();
  // //     CvSource outputStream = CameraServer.getInstance().putVideo("DriverCamera", 288, 216);
      
  // //     Mat source = new Mat();
  // //     Mat output = new Mat();
      
  // //     while(!Thread.interrupted()) {
  // //         cvSink.grabFrame(source);
  // //         Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
  // //         outputStream.putFrame(output);
  // //     }
  // // }).start();


  }

  public void robotPeriodic(){
    driveTrain.logs();
    elevator.logs();
    climbing.logs();
    //hatchIntake.logs();
    intakeLifting.logs();
    catchTarget.logs();
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

    Robot.catchTarget.detectionMode();

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
