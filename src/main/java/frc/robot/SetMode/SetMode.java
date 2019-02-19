package frc.robot.SetMode;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Subsystems.TFSubsystem;

public class SetMode<Mode_T> extends InstantCommand {

	TFSubsystem<Mode_T> subsystem;
	Mode_T mode;
	
	public SetMode(TFSubsystem<Mode_T> subsystem, Mode_T mode) {
		this.subsystem = subsystem;
		this.mode = mode;
	}
	
	@Override
	protected void execute() {
		subsystem.setMode(mode);
	}
}
