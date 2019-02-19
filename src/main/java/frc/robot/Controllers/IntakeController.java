/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.ProfilePoint;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

/**
 * Add your docs here.
 */
public class IntakeController implements DrivetrainController {
    Constants constants;
	
    double startT;	
	double linearActual, linearSetpoint, linearError;
    TrapezoidalMotionProfile profile;
    SpeedPID speedPID;

	public IntakeController(TrapezoidalMotionProfile profile) {
		constants = Constants.getConstants();
		
		
		this.profile = profile;
        refreshConstants();
        
        speedPID = new SpeedPID(Constants.intakePIDkP,Constants.intakePIDkI,Constants.intakePIDkD,Constants.intakePIDkF);
    }
    
    @Override
    public boolean update() {
        double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint point = profile.getAtTime(t);
		
		double feedforward = (point.vel * constants.IntakeProfile_kV) + (point.acc * constants.IntakeProfile_kA);
		
        linearActual = Robot.intakeLifting.getDistance();
		linearSetpoint = point.pos;
		double error = linearSetpoint-linearActual;
        double output = speedPID.calculate(linearSetpoint, linearActual);
        if(output > Constants.maxOutputIntake){
			output = Constants.maxOutputIntake;
		}else if(output < -Constants.maxOutputIntake){
			output = -Constants.maxOutputIntake;
		}
		
		linearError = error;
        Robot.intakeLifting.setSpeedIntake(output);
        SmartDashboard.putNumber("Output TrapezoidalIntake", output);
		return t >= profile.getDuration() && Math.abs(error) < constants.allowedIntakeError;
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

    @Override
    public void reset() {

    }

    @Override
    public void refreshConstants() {

    }
}
