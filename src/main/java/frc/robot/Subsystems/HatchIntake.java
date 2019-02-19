/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Commands.Intake.IntakeBall;


public class HatchIntake extends Subsystem {
  public final Compressor compressor = new Compressor();
  public final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PortMap.hatchSolenoidA, PortMap.hatchSolenoidB);
  public final VictorSP ballMotor = new VictorSP(PortMap.ballMotor);
  
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeBall());
  }

  public void hatchSolenoid(boolean open) {
    if(open){
      intakeSolenoid.set(Value.kForward);
    }else{
      intakeSolenoid.set(Value.kReverse);
    }
  }

  public void hatchOFF() {
      intakeSolenoid.set(Value.kOff);
  }

public void ballMotorSTART(){
  if(Robot.oi.getDriverJoystick().getRawAxis(2) != 0){
    ballMotor.set(Robot.oi.getDriverJoystick().getRawAxis(2)/2);
  }else if(Robot.oi.getDriverJoystick().getRawAxis(3) != 0){
    ballMotor.set(-Robot.oi.getDriverJoystick().getRawAxis(3)/2);
  }else{
    ballMotorSTOP();
  }
}

public void ballMotorSTOP(){
  ballMotor.set(0.0);
}

}
