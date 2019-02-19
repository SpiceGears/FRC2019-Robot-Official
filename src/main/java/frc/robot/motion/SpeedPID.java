/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.motion;
import frc.robot.Constants;
import frc.robot.SmartDashBoardInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpeedPID {
	
	double kP;
	double kI;
	double kD; 
	double kF;
	double setSpeed;
	double error;
	double accualSpeed;
	
	double lastT;
	double errSum;
	double prevError;
	
	boolean reset;

	double maxOut = 1;
	double minOut = -1;
	double maxI = 0;
	double minI = 0;
	
	Constants constants;
	
	public SpeedPID(double kP, double kI, double kD, double kF) {
		constants = Constants.getConstants();
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		
		if ( kI == 0 )
			maxI = 0;
		else
			maxI = 1/kI;
		minI = -maxI;
		
		reset();
	}

	public void setSetpoint(double setSpeed) {
		this.setSpeed = setSpeed;
	}
	
	public double getError() {
		if ( reset )
			return Double.MAX_VALUE;
		return error;
	}
	
	public double getInput() {
		return accualSpeed;
	}
	
	public void reset() {
		reset = true;
	}
	
	public double getSetpoint() {
		return setSpeed;
	}
	
	public void setOutputConstraints(double max, double min){
		maxOut = max;
		minOut = min;
	}
	
	public double calculate(double setSpeed, double accualSpeed) {
		double now = Timer.getFPGATimestamp();
		double dt = now-lastT;
		lastT = now;
		return calculate(dt, setSpeed, accualSpeed);
	}
	
	public double calculate(double dT, double setSpeed, double accualSpeed) {
		this.accualSpeed = accualSpeed;
		
		error = setSpeed - accualSpeed;
		
		errSum += error*dT;
		
		if ( errSum > maxI )
			errSum = maxI;
		else if ( errSum < minI )
			errSum = minI;

		double deltaPos = error-prevError;
		prevError = error;

		if ( reset ) {
			deltaPos = 0;
			lastT = Timer.getFPGATimestamp();
			prevError = 0;
			errSum = 0;
			reset = false;
		}
		
		double out = (kP*error) + (kI*errSum) + (kD*(deltaPos/dT)) + (kF*setSpeed);

		SmartDashboard.putNumber("kP", kP*error);
		SmartDashboard.putNumber("kI", kI*errSum);
		SmartDashboard.putNumber("kD", kD*(deltaPos/dT));
		
		if ( out > maxOut )
			out = maxOut;
		else if ( out < minOut )
			out = minOut;
		
		return out;
	}
}
