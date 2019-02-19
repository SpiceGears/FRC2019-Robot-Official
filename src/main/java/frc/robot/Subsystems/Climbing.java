/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Commands.Climbing.Climb;
import frc.robot.Commands.Climbing.ClimbingTrapez;
import frc.robot.Controllers.ClimbingController;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

/**
 * Add your docs here.
 */
public class Climbing extends Subsystem {
  private Spark climbingLeft;
  private Spark climbingRight;
  private VictorSP climbingDrive;
  private Encoder encoder;
  private ClimbingController climbingEncoder;
  private TrapezoidalMotionProfile profile;
  private Mode climbingMode = Mode.CLOSE;

  private double iChecker = 0;
  private double motorOutputSum = 0;
  private double setPoint = -0.001;
	private double newSetPoint = Constants.climbingMin;
  private double dt = 0;
  private double encoderDistance;
  private double lastT;
  double deltaEncoder;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ClimbingTrapez());
  }

  public enum Mode {
    OPEN,
    CLOSE
	}

  public Climbing(){
    climbingDrive = new VictorSP(PortMap.climbingDriveMotor);
    climbingLeft = new Spark(PortMap.climbingMotorLeft);
    climbingRight = new Spark(PortMap.climbingMotorRight);

    encoder = new Encoder(PortMap.climbingEncoderA, PortMap.climbingEncoderB);
    encoder.setDistancePerPulse(0.005);
    encoder.setReverseDirection(true);
    dt = 0;

  }

  public void update(){
	
	if(newSetPoint == setPoint) {
    SmartDashboard.putNumber("Debug_Climb", 3);
		climbingEncoder.update();
	} else {
      setPoint = newSetPoint;
			profile = new TrapezoidalMotionProfile(setPoint, Constants.ElevatorProfile_MaxV, Constants.ElevatorProfile_MaxA);
      climbingEncoder = new ClimbingController(profile);
      SmartDashboard.putNumber("Debug_Climb", 1);
  }
  
  switch(climbingMode) {
		case OPEN:
      newSetPoint = Constants.climbingMax;
      //SmartDashboard.putNumber("Debug_Climb", 0);
      break;
    case CLOSE:
      newSetPoint = Constants.climbingMin;
    default:
      break;
  }

  drive();
}

  public void climbingLinear(){
    climbingLeft.set(Robot.oi.getOperatorJoystick().getRawAxis(3));
    climbingRight.set(Robot.oi.getOperatorJoystick().getRawAxis(3));
  }

  public void stopClimb(){
    climbingLeft.set(0);
    climbingRight.set(0);
  }

  public void drive(){
    climbingDrive.set(Robot.oi.getDriverJoystick().getRawAxis(5));
  }

  public void setSpeed(double speed) {
    SmartDashboard.putNumber("Speed_climbing", speed);
    climbingLeft.set(-speed);
    climbingRight.set(-speed);

  }

  public void checkEncoder(){
    if(dt == 0){
        encoderDistance = getDistance();
      }
      double now = Timer.getFPGATimestamp();
      dt += now-lastT;
      lastT = now;
      iChecker++;
      motorOutputSum += climbingLeft.get();
      if(dt > 0.5){
        if(motorOutputSum/iChecker > 0.5){
            deltaEncoder = encoderDistance - getDistance();
          if(deltaEncoder >= 0.02 || deltaEncoder <= -0.02){
            setDefaultCommand(new Climb());
          }
        dt = 0;
        motorOutputSum = 0;
        iChecker = 0;  
      }
    }
  }

  public double getActualSpeed(){
    return (encoder.getRate());
  }

  public double getDistance(){
    return encoder.getDistance();
  }

  public void resetEncoder(){
    encoder.reset();
  }
  
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public Mode getMode(){
    return climbingMode;
  }
  
  public void setMode(Mode climbingMode) {
    this.climbingMode = climbingMode;
  }
  
  public void logs(){
    SmartDashboard.putNumber("ClimbingSpeed", climbingLeft.getSpeed());
    // SmartDashboard.putNumber("ClimbingActualSpeed", getActualSpeed());
    // SmartDashboard.putNumber("DeltaEncoder_Climbing", deltaEncoder);
    // SmartDashboard.putNumber("MotorOutputSum_Climbing", motorOutputSum);
    SmartDashboard.putNumber("ClimbingEncoder", encoder.getDistance());
  }
}
