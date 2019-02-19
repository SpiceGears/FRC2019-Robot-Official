/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Controllers;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.ProfilePoint;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

/**
 * Add your docs here.
 */
public class ProfileTurnController implements DrivetrainController {
	Constants constants;
	
	double startAngle;
	double startT;
    TrapezoidalMotionProfile profile;
    SpeedPID speedPID;
	
	double angularError, angularActual, angularSetpoint;
	
	/*
	 * ProfileTurnController constructor
	 * 
	 * @param profile The TrapezoidalMotionProfile to follow
	 * @param theta The angle to maintain while driving straight
	 */
	public ProfileTurnController(TrapezoidalMotionProfile profile) {
        constants = Constants.getConstants();
        speedPID = new SpeedPID(Constants.TurnPID_kP, Constants.TurnPID_kI, Constants.TurnPID_kD, Constants.TurnPID_kF);
		
		this.profile = profile;
		refreshConstants();
		start();
	}
	
	
	/*
	 * Configure this controller to begin following the profile, making t=0 the current moment
	 */
	public void start() {
		startT = Timer.getFPGATimestamp();
		startAngle = Robot.driveTrain.getHeading();
	}

	@Override
	public boolean update() {
		double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint point = profile.getAtTime(t);
		
		angularActual = Robot.driveTrain.getHeading();
		angularSetpoint = point.pos;
		
		double feedforward = (point.vel * constants.TurnProfile_kV) + (point.acc * constants.TurnProfile_kA);
		angularError = (angularActual-startAngle) - point.pos;
		double output = speedPID.calculate(angularSetpoint, angularActual);
		Robot.driveTrain.driveSet(output, -output);
		
		return t >= profile.getDuration();
	}


	@Override
	public void reset() {
		start();
	}

	@Override
	public void refreshConstants() {
		
	}


	@Override
	public double getLinearError() {
		return 0;
	}


	@Override
	public double getLinearActual() {
		return 0;
	}


	@Override
	public double getLinearSetpoint() {
		return 0;
	}


	@Override
	public double getAngularError() {
		return angularError;
	}


	@Override
	public double getAngularActual() {
		return angularActual;
	}


	@Override
	public double getAngularSetpoint() {
		return angularSetpoint;
	}
}