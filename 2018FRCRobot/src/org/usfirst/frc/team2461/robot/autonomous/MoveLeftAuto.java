package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.BoxManager;
import org.usfirst.frc.team2461.robot.SwerveDrive;

public class MoveLeftAuto extends MoveAuto implements AutoCode
{

	public MoveLeftAuto(SwerveDrive chassisIn, BoxManager boxMaangerIn) {
		super(chassisIn, boxMaangerIn);
	}

	@SuppressWarnings("static-access")
	@Override
	protected void prepareMoveSideCommand() {
		chassis.addAutoCommand(factory.command_MoveLeft(autoMoveSideDistance));
		
	}

	@Override
	protected String printMoveSideState() {
		return "Driving State: Moving Left";
	}
	
}
