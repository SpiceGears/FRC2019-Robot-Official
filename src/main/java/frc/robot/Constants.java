/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Constants {
	
	private static Constants instance;
	
	public static Constants getConstants() {		
		if ( instance == null ) {
				instance = new Constants();
		}
		return instance;
	}
	
	private String gameData = null;
	
	public static double MaxSpeedOnThirdStagedrive = 0.4;//0.3
	public static final double DRIVERotacion_kP = 0.25;
	public static final double DRIVERotacion_kP_Vison = 0.013;
	public static double maxTurnValue = 0.35;
	
	//Drive
		
		public static final double kDriveVelocityKp = 0.9;
		public static final double kDriveVelocityKi = 0;
		public static final double kDriveVelocityKd = 10.0;
		public static final double kDriveVelocityKf = 0;
		public static final int kDriveVelocityIZone = 0;
		public static final int kLongCANTimeoutMs = 10;

		public static double DriveProfile_kV = 0.119;
		public static double DriveProfile_kA = 0;
		public static double DriveProfile_kP = 1;
		public static double DriveProfile_theta_kP = 0.03;
		public static double ToleranceDriveError = 0.1;

		public static double SpeedDrivePIDkP = 0.2;
		public static double SpeedDrivePIDkI = 0.04;
		public static double SpeedDrivePIDkD = 0.005;
		public static double SpeedDrivePIDkF = 0.1;
		
		public static double Gyro_kP = 0.19;
		
		public static double TurnProfile_kV = 0.02;
		public static double TurnProfile_kA = 0;
		public static double TurnProfile_kP = -0.049;
		
		public static double TurnPID_kP = 0.045; // 0.059
		public static double TurnPID_kI = 0.00000015; // 0
		public static double TurnPID_kD = 0.0045; // 0.0045
		public static double TurnPID_kF = 0;
		
		public static double MaxDriveVelocity = 8000;
		
		public static double LeftDriveVelocityP = 0.3;
		public static double LeftDriveVelocityD = 0;
		public static double LeftDriveVelocityF = 1023.0 / MaxDriveVelocity;
		
		public static double RightDriveVelocityP = 0.3;
		public static double RightDriveVelocityD = 0;
		public static double RightDriveVelocityF = 1023.0 / MaxDriveVelocity;
		
		public static double MaxDriveVelocityLow = 3000;
		
		public static double LeftDriveVelocityPLow = 0;
		public static double LeftDriveVelocityDLow = 0;
		public static double LeftDriveVelocityFLow = 1023.0 / MaxDriveVelocityLow;

		public static double RightDriveVelocityPLow = 0;
		public static double RightDriveVelocityDLow = 0;
		public static double RightDriveVelocityFLow = 1023.0 / MaxDriveVelocityLow;
		
		public static double DriveRampRate = 120;
		
		
		public static double PivotTurnPID_kP = 0.003; //0.008
		public static double PivotTurnPID_kI = 0.0002; //0.0003
		public static double PivotTurnPID_kD = 0.003; //0.004

		public static double AllowedDriveError = 0.25; // 0.25
		public static double AllowedTurnError = 1.8;
		public static double AllowedTurnPIDError = 1.3;
		public static double AllowedPivotTurnError = 4.0;

	//Elevator
		public static double elevatorThirdStage = 0.62;
		public static double elevatorSecondStage = 0.46;
		public static double elevatorFirstStage = 0.18;
		public static double elevatorIntake = 0.0;	//0.045;
		
		public static double elevator_kP = 4;
		public static double elevator_kI = 0.15;
		public static double elevator_kD = 0.005;
		public static double elevator_kF = 0.1;
		public static double maxOutputElevator = 0.4;
		
		public static double allowedElevatorError = 0.03;
		public static double cutTrapezBeforeEnd = 0.1;
		public static double ElevatorProfile_kV = 0.5;
		public static double ElevatorProfile_kA = 0;
		public static double ElevatorProfile_kP = 1;
		public static double ElevatorProfile_theta_kP = 0.03;
		
		public static double ElevatorProfile_MaxV = 1;
		public static double ElevatorProfile_MaxA = 2;

	//IntakeLift

		public static double allowedIntakeError = 0.02;
		public static double IntakeProfile_kV = 0.5;
		public static double IntakeProfile_kA = 0.0;
		public static double IntakeProfile_kP = 0.7;
		public static double IntakeProfile_theta_kP = 0.03;
		public static double maxOutputIntake = 0.7;
				
		public static double IntakeProfile_MaxV = 2;
		public static double IntakeProfile_MaxA = 0.7;
		
		public static double intakeZeroDeg = 1.5;
		public static double intake45_UP = 7;
		public static double intake90_UP = 12.25;
		public static double intake135_UP = 14;
		public static double intake150_UP = 18.5;
				
		public static double intakePIDkP = 0.7;
		public static double intakePIDkI = 0.07;
		public static double intakePIDkD = 0.06;
		public static double intakePIDkF = 0.0;

		public static double turnTrapezoidalMultiply = 1.3;
		//Climbing

		public static double climbingPIDkP = 0.2;
		public static double climbingPIDkI = 0.01;
		public static double climbingPIDkD = 0.005;
		public static double climbingPIDkF = 0.0;

		public static double climbingMax = 14.5;
		public static double climbingMin = 0;
		public static double maxOutputClimb = 0.8;

		public static double maxElevatorVelocity = 0.1;

		public static double allowedClimbingError = 1;

		public String getGameData() {
			return gameData;
		}
		public void setGameData(String gameData) {
			this.gameData = gameData;
		}

}
