package frc.robot.Controllers;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.ProfilePoint;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorController implements DrivetrainController {
	Constants constants;
	
	double startT;
	TrapezoidalMotionProfile profile;
	SpeedPID speedPID;
	
	double linearActual, linearSetpoint, linearError;
	public ElevatorController(TrapezoidalMotionProfile profile) {
		constants = Constants.getConstants();
		
		
		this.profile = profile;
		refreshConstants();

		speedPID = new SpeedPID(Constants.elevator_kP, Constants.elevator_kI, Constants.elevator_kD, Constants.elevator_kF);
	}

	/*
	 * Configure this controller to begin following the profile, making t=0 the current moment.
	 * @param theta The desired angle for drivestraight correction
	 */
	
	@Override
	public boolean update() {
		double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint point = profile.getAtTime(t);
		
		double feedforward = (point.vel * constants.ElevatorProfile_kV) + (point.acc * constants.ElevatorProfile_kA);
		
		linearActual = Robot.elevator.getDistance();
		linearSetpoint = point.pos;
		double error = linearSetpoint-linearActual;
		double output = speedPID.calculate(linearSetpoint, linearActual);
		if(output > Constants.maxOutputElevator){
			output = Constants.maxOutputElevator;
		}else if(output < -Constants.maxOutputElevator){
			output = -Constants.maxOutputElevator;
		}
		
		linearError = error;
		Robot.elevator.setSpeedElevator(output);
		SmartDashboard.putNumber("Output TrapezoidalElevator", output);
		
		return t >= profile.getDuration() && Math.abs(error)+Constants.cutTrapezBeforeEnd < constants.allowedElevatorError;
	}

	public void reset() {
		startT = Timer.getFPGATimestamp();
	}

	@Override
	public void refreshConstants() {
	}

	@Override
	public double getLinearError() {
		return linearError;
	}

	@Override
	public double getLinearActual() {
		return linearActual;
	}

	@Override
	public double getLinearSetpoint() {
		return linearSetpoint;
	}

	@Override
	public double getAngularError() {
		return 0;
	}

	@Override
	public double getAngularActual() {
		return 0;
	}

	@Override
	public double getAngularSetpoint() {
		return 0;
	}
}

