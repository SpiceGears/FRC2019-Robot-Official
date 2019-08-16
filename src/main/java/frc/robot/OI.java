/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.prefs.PreferenceChangeEvent;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Climbing.Climb;
import frc.robot.Commands.Climbing.ClimbingTrapez;
import frc.robot.Commands.Climbing.OpenClimbDriveTrain;
import frc.robot.Commands.Drive.DriveToTarget;
import frc.robot.Commands.Elevator.ElevatorJoystick;
import frc.robot.Commands.Elevator.ElevatorSetFirstStage;
import frc.robot.Commands.Elevator.ElevatorSetIntake;
import frc.robot.Commands.Elevator.ElevatorSetSecondStage;
import frc.robot.Commands.Elevator.ElevatorSetThirdStage;
import frc.robot.Commands.Elevator.ElevatorTrapez;
import frc.robot.Commands.Intake.HatchShooter;
import frc.robot.Commands.Intake.Intake0Deg;
import frc.robot.Commands.Intake.Intake135UP;
import frc.robot.Commands.Intake.Intake150;
// import frc.robot.Commands.Intake.DropBall;
// import frc.robot.Commands.Intake.IntakeBall;
// import frc.robot.Commands.Intake.IntakeClose;
// import frc.robot.Commands.Intake.IntakeOpen;
import frc.robot.Commands.Intake.Intake45UP;
import frc.robot.Commands.Intake.Intake90UP;
import frc.robot.Commands.Intake.IntakeBall;
import frc.robot.Commands.Intake.IntakeClose;
import frc.robot.Commands.Intake.IntakeOpen;
import frc.robot.Commands.Intake.PushHatch;
import frc.robot.Subsystems.HatchIntake;

public class OI {

	private Joystick driver;
	private Joystick operator;
	private JoystickButton openIntake;
	private JoystickButton closeIntake;
	private JoystickButton ballIntake;
	private JoystickButton ballDrop;
	private JoystickButton firstStage;
	private JoystickButton secondStage;
	private JoystickButton thirdStage;
	private JoystickButton intakeStage;
	private JoystickButton intake150up;
	private JoystickButton intake135up;
	private JoystickButton intake90up;
	private JoystickButton intake45up;
	private JoystickButton intake0;
	private JoystickButton runPathfinder;
	private JoystickButton setElevatorControl;
	private JoystickButton setElevatorTrapezoidal;
	private JoystickButton secondDriveTrain;
	private JoystickButton setManualClimb;
	private JoystickButton setTrapezClimb;

    public OI() {
		driver = new Joystick(0);
		operator = new Joystick(1);

		DriverButtons driverButtons;

		// openIntake  = new JoystickButton(driver, DriverButtons.openIntake);
		// openIntake.whenPressed(new HatchShooter());

		// openIntake  = new JoystickButton(driver, DriverButtons.openIntake);
		// openIntake.whenPressed(new IntakeOpen());

		// openIntake  = new JoystickButton(driver, DriverButtons.closeIntake);
		// openIntake.whenPressed(new IntakeClose());

		// ballIntake = new JoystickButton(driver, DriverButtons.ballMotorStart);
		// ballIntake.whileHeld(new IntakeBall());

		// ballDrop = new JoystickButton(driver, DriverButtons.ballMotorStop);
		// ballDrop.whileHeld(new DropBall());

		firstStage = new JoystickButton(operator, DriverButtons.elevatorFirstStage);
		firstStage.whenPressed(new ElevatorSetFirstStage());

		secondStage = new JoystickButton(operator, DriverButtons.elevatorSecondStage);
		secondStage.whenPressed(new ElevatorSetSecondStage());

		thirdStage = new JoystickButton(operator, DriverButtons.elevatorThirdStage);
		thirdStage.whenPressed(new ElevatorSetThirdStage());

		intakeStage = new JoystickButton(operator, DriverButtons.elevatorIntake);
		intakeStage.whenPressed(new ElevatorSetIntake());

		// secondDriveTrain = new JoystickButton(operator, DriverButtons.secondDriveTrain);
		// secondDriveTrain.whenPressed(new OpenClimbDriveTrain());

		// setManualClimb = new JoystickButton(operator, DriverButtons.setManualControlClimb);
		// setManualClimb.whenPressed(new Climb());

		// setTrapezClimb = new JoystickButton(operator, DriverButtons.setTrapezControl2);
		// setTrapezClimb.whenPressed(new ClimbingTrapez());

		setElevatorControl = new JoystickButton(operator, DriverButtons.setManualControl);
		setElevatorControl.whenPressed(new ElevatorJoystick());

		setElevatorTrapezoidal = new JoystickButton(operator, DriverButtons.setTrapezControl);
		setElevatorTrapezoidal.whenPressed(new ElevatorTrapez());
    }

	public Joystick getDriverJoystick() {
		return driver;
	}

	public Joystick getOperatorJoystick() {
		return operator;
	}

}
