/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PortMap {
	
	//Ports
	
	//Drive Motors
	public static int kLeftMaster = 0;
	public static int kRightMaster = 1;
	public static int kLeftSlave = 2;
	public static int kRightSlave = 3;
	 
	//Elevator
	public static int elevatorMotorF = 0;
	public static int elevatorMotorS = 1;
	public static int elevatorMotorT = 6;
	public static int elevatorEncoderA = 2;
	public static int elevatorEncoderB = 3;

	//Hatch Intake
	public static int hatchSolenoidA = 0;
	public static int hatchSolenoidB = 1;

	//IntakeBalls
	public static int ballMotor = 5;

	//IntakeLifting
	public static int liftMotorR = 0;
	public static int liftMotorL = 1;
	public static int intakeEncoderA = 0;
	public static int intakeEncoderB = 1;

	//Climbing
	public static int climbingMotorRight = 3;
	public static int climbingMotorLeft = 2;
	public static int climbingDriveMotor = 4;
	public static int climbingEncoderA = 4;
	public static int climbingEncoderB = 5;
	 
	 
}
