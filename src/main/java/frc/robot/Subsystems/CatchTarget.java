/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Controllers.ProfileTurnController;
import frc.robot.motion.TrapezoidalMotionProfile;

/**
 * Add your docs here.
 */
public class CatchTarget extends Subsystem {
  // NetworkTableEntry getDistance;
  private NetworkTableEntry getYaw;
  private NetworkTableEntry getCargoYaw;
  private NetworkTableEntry getTape;
  private NetworkTableEntry getCargo;
  private NetworkTableEntry mode;

  private double yaw;
  private double cargoYaw;
  private boolean tape;
  private boolean cargo;
  private TrapezoidalMotionProfile profile;
  private ProfileTurnController turnController;


  @Override
  public void initDefaultCommand() {
    
  }

  public CatchTarget(){
    
  }

  // public void update(double setPoint){
  //       profile = new TrapezoidalMotionProfile(setPoint, Constants.TurnProfile_kV, Constants.TurnProfile_kA);
  //       turnController = new ProfileTurnController(profile);
  // }

  public double getYaw(){
    return yaw;
  }

  public boolean isTapeDetected(){
    return tape;
  }

  public double getCargoYaw(){
    return cargoYaw;
  }

  public boolean isCargoDetected(){
    return cargo;
  }

  public void detectionMode(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("ChickenVision");
    getYaw = table.getEntry("tapeYaw");
    getTape = table.getEntry("tapeDetected");
    getCargoYaw = table.getEntry("cargoYaw");
    getCargo = table.getEntry("cargoDetected");
    mode = table.getEntry("Tape");
    inst.startClientTeam(5883);
    inst.startDSClient();
    
    yaw = getYaw.getDouble(0.0);
    tape = getTape.getBoolean(false);
    cargoYaw = getCargoYaw.getDouble(0.0);
    cargo = getCargo.getBoolean(false);

    // if(Robot.oi.getDriverJoystick().getRawButton(2)){
    //   mode.setBoolean(false);
    // }else{
    //   mode.setBoolean(true);
    // }

    getYaw();
    isTapeDetected();

    if(!isTapeDetected()){
      yaw = 0;
    }
    
  }

  public void logs(){
    SmartDashboard.putNumber("TapeYaw", yaw);
    SmartDashboard.putBoolean("TapeDetected", tape);
  }
}
