/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Commands.Intake.IntakeOpen;


public class HatchIntake extends Subsystem {
  public final Compressor compressor = new Compressor();
  public final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PortMap.hatchSolenoidA, PortMap.hatchSolenoidB);
 
  public void initDefaultCommand() {
    //setDefaultCommand(new IntakeOpen());
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

}
