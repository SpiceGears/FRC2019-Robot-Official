/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Commands.Elevator.ElevatorJoystick;
import frc.robot.Commands.Elevator.ElevatorTrapez;
import frc.robot.Controllers.ElevatorController;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;



public class Elevator extends Subsystem {
	
	public enum Mode {
		INTAKE,
		STAGE_ONE, 
		STAGE_TWO,
		STAGE_THREE
	}
	
	private Mode elevatorMode = Mode.INTAKE;
	private VictorSPX elevatorMotorF;
	private VictorSPX elevatorMotorS;
	private VictorSP elevatorMotorT;
	private Encoder encoder;
	private ElevatorController elevatorController;
	private TrapezoidalMotionProfile profile;

	private double lastT;
	private double dt = 0;
  	private double speed;
	private double encoderDistance;
	private double setPoint = -0.001;
	private double newSetPoint = Constants.elevatorIntake;
	private double motorOutputSum = 0;
	private double iChecker = 0;
	private double deltaEncoder;

	public Elevator(){
		elevatorMotorF = new VictorSPX(PortMap.elevatorMotorF);
		elevatorMotorS = new VictorSPX(PortMap.elevatorMotorS);
		elevatorMotorT = new VictorSP(PortMap.elevatorMotorT);
		elevatorMotorF.configFactoryDefault();
		elevatorMotorS.configFactoryDefault();
		encoder = new Encoder(PortMap.elevatorEncoderA, PortMap.elevatorEncoderB);
    	encoder.setDistancePerPulse(0.00005);
    	encoder.setReverseDirection(false);
		lastT = 0;
		speed = 0;
		dt = 0;
	}

  
public void initDefaultCommand(){
   setDefaultCommand(new ElevatorTrapez());
}

public void update(){
	
	if(newSetPoint == setPoint) {
		elevatorController.update();
	} else {
		if(elevatorMotorF.getMotorOutputPercent() < 0.2){
			setPoint = newSetPoint;
			profile = new TrapezoidalMotionProfile(setPoint, Constants.ElevatorProfile_MaxV, Constants.ElevatorProfile_MaxA);
			elevatorController = new ElevatorController(profile);
		}
	}
	
	if(-Robot.oi.getOperatorJoystick().getRawAxis(1) > 0.1){
		newSetPoint = newSetPoint + -Robot.oi.getOperatorJoystick().getRawAxis(1)/15;
	} else if(-Robot.oi.getOperatorJoystick().getRawAxis(1) < -0.1) {
		newSetPoint = newSetPoint + -Robot.oi.getOperatorJoystick().getRawAxis(1)/15;
	} else {
		//newSetPoint = setPoint;
	}

	switch(elevatorMode) {
		case STAGE_ONE:
			newSetPoint = Constants.elevatorFirstStage;
			break;
		case STAGE_TWO:
			newSetPoint = Constants.elevatorSecondStage;
			break;
		case STAGE_THREE:
			newSetPoint = Constants.elevatorThirdStage;
			break;
		case INTAKE:
			newSetPoint = Constants.elevatorIntake;
			break;
		default:
			//setPoint = constants.elevatorIntake;
			break;
	}
}

public void controlMotor() {
    	
	double speed = -Robot.oi.getOperatorJoystick().getRawAxis(1);	
	// if (speed > 0) {
	// 	speed *= 1;
	// }
	elevatorMotorF.set(ControlMode.PercentOutput, speed*Constants.maxOutputElevator);
	elevatorMotorS.set(ControlMode.PercentOutput, speed*Constants.maxOutputElevator);
	elevatorMotorT.set(speed*Constants.maxOutputElevator);
}

public void setMotorPower(double power){
	elevatorMotorF.set(ControlMode.PercentOutput, power);
	elevatorMotorS.set(ControlMode.PercentOutput, power);
	elevatorMotorT.set(power);
}

public void setSpeedElevator(double speed){
	elevatorMotorF.set(ControlMode.PercentOutput, speed);
	elevatorMotorS.set(ControlMode.PercentOutput, speed);
	elevatorMotorT.set(speed);
	checkEncoder();
}

public void stopElevator(){
    elevatorMotorF.set(ControlMode.PercentOutput, 0.0);
	elevatorMotorS.set(ControlMode.PercentOutput, 0.0);
	elevatorMotorT.set(0);

}

public double getDistance(){
	return encoder.getDistance();
}

public void resetEncoder(){
	encoder.reset();
}

public Mode getMode(){
	return elevatorMode ;
}

public void setMode(Mode elevatorMode) {
	this.elevatorMode = elevatorMode;
}

public double getSetPoint() {
	return setPoint;
}

public void setSetPoint(double setPoint) {
	this.setPoint = setPoint;
}

public double getActualSpeed(){
	return encoder.getRate();
}

public void checkEncoder(){
	if(dt == 0){
		encoderDistance = getDistance();
	  }
	  double now = Timer.getFPGATimestamp();
	  dt += now-lastT;
	  lastT = now;
	  iChecker++;
	  motorOutputSum += elevatorMotorF.getMotorOutputPercent();
	  if(dt > 0.5){
		  if(motorOutputSum/iChecker > Constants.maxOutputElevator - 0.1){
			  	deltaEncoder = encoderDistance - getDistance();
		  	if(Math.abs(deltaEncoder) == 0 || Math.abs(getDistance()) > 1){
				setDefaultCommand(new ElevatorJoystick());
		  	}
			dt = 0;
			motorOutputSum = 0;
			iChecker = 0;  
		}
	}
}

  public void logs(){
	SmartDashboard.putNumber("ElevatorEncoder", encoder.getDistance());
	SmartDashboard.putNumber("ActualSpeedElevator", getActualSpeed());
	SmartDashboard.putNumber("ElevatorOutput", elevatorMotorF.getMotorOutputPercent());
	SmartDashboard.putNumber("ElevatorNewSetPoint", newSetPoint);
	SmartDashboard.putNumber("DeltaEncoder_Elevator", deltaEncoder);
	SmartDashboard.putNumber("MotorOutputSum_Elevator", motorOutputSum);
	SmartDashboard.putNumber("dt", dt);
	SmartDashboard.putNumber("ElevatorSpeed",speed);
  }
}
