/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Commands.Intake.IntakeBall;

/**
 * Add your docs here.
 */
public class BallIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public final VictorSP ballMotor = new VictorSP(PortMap.ballMotor);
  

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeBall());
  }

  
public void ballMotorSTART(){
  if(Robot.oi.getDriverJoystick().getRawAxis(2) != 0){
    ballMotor.set(-Robot.oi.getDriverJoystick().getRawAxis(2)/1.8);
  }else if(Robot.oi.getDriverJoystick().getRawAxis(3) != 0){
    ballMotor.set(Robot.oi.getDriverJoystick().getRawAxis(3)/1.8);
    System.out.print("ball : " + Robot.oi.getDriverJoystick().getRawAxis(3));

  }else{
    ballMotorSTOP();
  }
}

public void ballMotorSTOP(){
  ballMotor.set(0.0);
}
}
