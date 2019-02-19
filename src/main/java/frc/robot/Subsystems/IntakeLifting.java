
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
import frc.robot.Commands.Intake.IntakeLift;
import frc.robot.Commands.Intake.IntakeTrapez;
import frc.robot.Controllers.IntakeController;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

/**
 * Add your docs here.
 */
public class IntakeLifting extends Subsystem {
  public final Spark liftMotorF;
  public final Spark liftMotorS;
  public Encoder intakeEncoder;

  private IntakeController intakeController;
  private TrapezoidalMotionProfile profile;
  private Mode intakeMode = Mode.DEG_0;
  private SpeedPID speedPID;
  
  private boolean isCheckEncoder = true;
  private double lastT;
  private double speed;
	private double encoderDistance;
	private double dt = 0;
	private double setPoint = -0.001;
  private double newSetPoint = Constants.intakeZeroDeg;
  private double iChecker = 0;
  private double motorOutputSum = 0;
  private double deltaEncoder;
  private boolean check = false;

  public enum Mode { //Sprawdzić odczyty encodera jak nam oddadzą robota 
		DEG_0,
    DEG_45_UP,
    DEG_90_UP,
    DEG_135_UP,
    DEG_150_UP
  }
  
  public IntakeLifting(){
    speedPID = new SpeedPID(Constants.intakePIDkP,Constants.intakePIDkI,Constants.intakePIDkD,Constants.intakePIDkF);
    liftMotorF = new Spark(PortMap.liftMotorR);
    liftMotorS = new Spark(PortMap.liftMotorL);
    intakeEncoder = new Encoder(PortMap.intakeEncoderA , PortMap.intakeEncoderB);

    intakeEncoder.setDistancePerPulse(0.05);
    intakeEncoder.setReverseDirection(false);
    intakeEncoder.setSamplesToAverage(7);
    liftMotorS.setInverted(true);
    lastT = 0;
		speed = 0;
		dt = 0;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeTrapez());
  }

  public void update(){

    if(newSetPoint == setPoint) {
      intakeController.update();
    } else {
      setPoint = newSetPoint;
      profile = new TrapezoidalMotionProfile(setPoint, Constants.IntakeProfile_MaxV, Constants.IntakeProfile_MaxA);
      intakeController = new IntakeController(profile);
    }
    
    if(Robot.oi.getOperatorJoystick().getRawAxis(3) > 0) {
      newSetPoint = newSetPoint + Robot.oi.getOperatorJoystick().getRawAxis(3)/100;
    }else if(Robot.oi.getOperatorJoystick().getRawAxis(3) < 0) {
      newSetPoint = newSetPoint + Robot.oi.getOperatorJoystick().getRawAxis(3)/100;
    }else {
      //newSetPoint = setPoint;
    }

    if(Robot.oi.getOperatorJoystick().getRawButton(7) && Robot.oi.getOperatorJoystick().getRawButton(5)){
      newSetPoint = (Constants.intakeZeroDeg+Constants.intake45_UP)/2;
    }else if(Robot.oi.getOperatorJoystick().getRawButton(5) && Robot.oi.getOperatorJoystick().getRawButton(6)){
      newSetPoint = (Constants.intake90_UP+Constants.intake45_UP)/2;
    }else if(Robot.oi.getOperatorJoystick().getRawButton(6) && Robot.oi.getOperatorJoystick().getRawButton(8)){
      newSetPoint = (Constants.intake90_UP+Constants.intake135_UP)/2;
    }else if(Robot.oi.getOperatorJoystick().getRawButton(7)){
      newSetPoint = Constants.intakeZeroDeg;
    }else if(Robot.oi.getOperatorJoystick().getRawButton(5)){
      newSetPoint = Constants.intake45_UP;
    }else if(Robot.oi.getOperatorJoystick().getRawButton(6)){
      newSetPoint = Constants.intake90_UP;
    }else if(Robot.oi.getOperatorJoystick().getRawButton(8)){
      newSetPoint = Constants.intake135_UP;
    }else if(Robot.oi.getOperatorJoystick().getRawButton(15)){
      newSetPoint = Constants.intake150_UP;
    }
    
    // switch(intakeMode) {
    // case DEG_0:
    //   newSetPoint = Constants.intakeZeroDeg;
    //   break;
    // case DEG_45_UP:
    //   newSetPoint = Constants.intake45_UP;
    //   break;
    // case DEG_90_UP:
    //   newSetPoint = Constants.intake90_UP;
    //   break;
    // case DEG_135_UP:
    //   newSetPoint = Constants.intake135_UP;
    //   break;
    //   case DEG_150_UP:
    //   newSetPoint = Constants.intake150_UP;
    //   break;
    // default:
    //   break;
    // }

    
    checkEncoder();
  }

  public void liftIntake(){
      liftMotorF.set(Robot.oi.getOperatorJoystick().getRawAxis(3));
      liftMotorS.set(Robot.oi.getOperatorJoystick().getRawAxis(3));
  }
  

  public void setSpeedIntake(double speed) {
        liftMotorF.set(speed);
        liftMotorS.set(speed);

  }
  
  public void intakeStop(){
    liftMotorF.set(0.0);
    liftMotorS.set(0.0);
  }

  public void checkEncoder(){
    if(isCheckEncoder){
      if(dt == 0){
        encoderDistance = getDistance();
      }
      double now = Timer.getFPGATimestamp();
      dt += now-lastT;
      lastT = now;
      iChecker++;
      motorOutputSum += liftMotorF.get();
      if(dt > 0.5){
        if(motorOutputSum/iChecker > 0.55){
            deltaEncoder = encoderDistance - getDistance();
          if(Math.abs(deltaEncoder) == 0 || Math.abs(getDistance()) > 30){
            setDefaultCommand(new IntakeLift());
            check = true;
          }
      }
      dt = 0;
      motorOutputSum = 0;
      iChecker = 0;  
    }
    }
  }

  public void resetEncoder(){
    intakeEncoder.reset();
  }

  public double getDistance(){
    return intakeEncoder.getDistance();
  }

  public void setIsCheckEncoder(boolean isEncoder){
    isCheckEncoder = isEncoder;
  }
  
  public Mode getMode() {
    return intakeMode;
  }
  
  public void setMode(Mode intakeMode) {
    this.intakeMode = intakeMode;
  }
  
  public double getSetPoint() {
    return setPoint;
  }
  
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getActualSpeed(){
    return intakeEncoder.getRate()/4096;
  }

  public void logs(){
    SmartDashboard.putNumber("IntakeEncoder", intakeEncoder.getDistance());
    // SmartDashboard.putNumber("DeltaEncoder", deltaEncoder);
    SmartDashboard.putNumber("IntakeRawOutput", liftMotorF.get());
    // SmartDashboard.putNumber("IntakeNewSetpoint_Intake", newSetPoint);
    SmartDashboard.putBoolean("Czytodziala", check);
    }
}
