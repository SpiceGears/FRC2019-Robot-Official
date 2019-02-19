/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Commands.Drive.Drive;
import frc.robot.Commands.Drive.DriveToTarget;
import frc.robot.Controllers.ProfileTurnController;
import frc.robot.drivers.TalonSRXFactory;
import frc.robot.motion.TrapezoidalMotionProfile;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private TalonSRX lMaster, rMaster, lSlave, rSlave;
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro(kGyroPort);
  private TrapezoidalMotionProfile profile;
  private ProfileTurnController turnController;

  
  private static final int kVelocityControlSlot = 0;
  private long oldTime = 0;
  private double speed = 0;
  private double speedAddicional = 0;
  private double deltaTime = 0;
  private double turn;
  double startTime;


  @Override
  public void initDefaultCommand(){
    setDefaultCommand(new Drive());
  }

  private void configureMaster(TalonSRX talon, boolean left){
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
            .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
    if (sensorPresent != ErrorCode.OK) {
        DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
    }
    talon.setSensorPhase(true);
    talon.enableVoltageCompensation(true);
    // talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
    // talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
    // talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
    // talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
    talon.configNeutralDeadband(0.04, 0);
}
  public void driveSet(double leftMotor, double rightMotor){
    rMaster.set(ControlMode.PercentOutput, rightMotor);
    lMaster.set(ControlMode.PercentOutput, leftMotor);
  }

  public DriveTrain(){
        lMaster = TalonSRXFactory.createDefaultTalon(PortMap.kLeftMaster);
        configureMaster(lMaster, false);

        lSlave = TalonSRXFactory.createPermanentSlaveTalon(PortMap.kLeftSlave,
        PortMap.kLeftMaster);
        lSlave.setInverted(false);

        rMaster = TalonSRXFactory.createDefaultTalon(PortMap.kRightMaster);
        configureMaster(rMaster, false);

        rSlave = TalonSRXFactory.createPermanentSlaveTalon(PortMap.kRightSlave,
        PortMap.kRightMaster);
      
        gyro.calibrate();
        resetGyro();
      
  }
  public synchronized void reloadGains() {
    lMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
    lMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
    lMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
    lMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
    lMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);

    rMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
    rMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
    rMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
    rMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
    rMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);
}
  

public void update(double setPoint){
  profile = new TrapezoidalMotionProfile(setPoint, Constants.TurnProfile_kV, Constants.TurnProfile_kA);
  turnController = new ProfileTurnController(profile);
}

public void drive(){
      if(Robot.oi.getDriverJoystick().getRawAxis(1) != 0 || Robot.oi.getDriverJoystick().getRawAxis(4) != 0){
        long newTime = System.currentTimeMillis();
        deltaTime = newTime - oldTime;
        double joySpeedValue = Robot.oi.getDriverJoystick().getRawAxis(1);
        //SmartDashboard.putNumber("encoderRight", getDistanceInMeters());

        speedAddicional = 0.1;
        speedAddicional = (deltaTime/50) * speedAddicional;
        
        if(Math.abs(joySpeedValue) < 0.2){
          joySpeedValue = 0;
        }
        
        if(-joySpeedValue > speed){
          speed = speed + speedAddicional;
          oldTime = System.currentTimeMillis();
        } else if (-joySpeedValue < speed){
          speed = speed - speedAddicional; 
          oldTime = System.currentTimeMillis();
        } else {
          speed = speed;
          oldTime = System.currentTimeMillis();
        }
        
        if (speed > 1){
          speed = 1;
        }
        else if
          (speed < -1){
          speed = -1;
        }
        
        if(Robot.oi.getDriverJoystick().getRawButton(1) && Robot.catchTarget.isTapeDetected()){
          turn = Robot.catchTarget.getYaw()*Constants.DRIVERotacion_kP_Vison;
          if(turn > 0.18){
            turn = 0.18;
          }else if(turn < 0.09 && turn > 0){
            turn = 0.09;
          }else if(turn > -0.09 && turn < 0){
            turn = -0.09;
          }else if(turn < -0.18){
            turn = -0.18;
          }

        }else{
          turn = Robot.oi.getDriverJoystick().getRawAxis(4) * Constants.DRIVERotacion_kP;
          if(Math.abs(turn) < 0.1){
            turn = 0;
          }
        }        
        
        if(Math.abs(speed) < 0.2) {
          turn *= 1;//2.33 
        }
        
        lMaster.set(ControlMode.PercentOutput, speed + turn);
        rMaster.set(ControlMode.PercentOutput, -(speed - turn));

      }
    }

    public void stopDrive(){
      lMaster.set(ControlMode.PercentOutput, 0.0);
      rMaster.set(ControlMode.PercentOutput, 0.0);
      speed = 0;
      speedAddicional = 0;
      deltaTime = 0;
    }

    public void resetSpeed(){
      speed = 0;
    }

    public double getLeftActualSpeed(){
      return ((double)(lMaster.getSelectedSensorVelocity())/4096.0) * 0.4788;
    }

    public double getRightActualSpeed(){
      return -((double) rMaster.getSelectedSensorVelocity()/4096) * 0.4788;
    }

    public double getHeading(){
      return gyro.getAngle();
    }

    public void resetGyro(){
      gyro.reset();
    }

    public void logs(){
      SmartDashboard.putNumber("DrivePercentOutput", speed);
      SmartDashboard.putNumber("DriveSpeedAddicional", speedAddicional);
      SmartDashboard.putNumber("DriveEncoderLeft", lMaster.getSelectedSensorPosition());
      SmartDashboard.putNumber("DriveEncoderRight", rMaster.getSelectedSensorPosition());
      SmartDashboard.putNumber("ActualSpeedL", getLeftActualSpeed());
      SmartDashboard.putNumber("ActualSpeedR", getRightActualSpeed());
      SmartDashboard.putNumber("ActualLeftSpeed",speed + turn);
      SmartDashboard.putNumber("ActualRightSpeed",speed - turn);
       SmartDashboard.putNumber("Gyro", getHeading());

    }
    
}

