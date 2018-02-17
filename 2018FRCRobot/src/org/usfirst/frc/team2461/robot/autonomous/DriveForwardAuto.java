package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;
import org.usfirst.frc.team2461.robot.SwerveDriveAutoCommandFactory;

public class DriveForwardAuto implements AutoCode
{
	private enum State {
		BEGIN, DRIVE_FORWARD, STOP
	}
	
	private State autoState;
	private SwerveDrive chassis;
	private SwerveDriveAutoCommandFactory factory = SwerveDriveAutoCommandFactory.getInstance();
	
	private double autoLineDistance = 60; //set to 60inches for testing purposes
	private double futureTime;
	
	public DriveForwardAuto(SwerveDrive driveTrain)
	{
		chassis = driveTrain;
		autoState = State.BEGIN;
	}
	
	public void run() {
		switch(autoState) {
			case BEGIN:
				begin();
				break;
			case DRIVE_FORWARD:
				driveForward();
				break;
			case STOP:
				stop();
				break;
			default:
				break;
			
		}
	}
	
	@SuppressWarnings("static-access")
	private void begin() {
		chassis.clearAutoCommands();
		chassis.addAutoCommand(factory.command_GoForward(autoLineDistance));
		chassis.driveAuto();
		autoState = State.DRIVE_FORWARD;
		futureTime = Robot.timer.get() + 0.1;
	}
	
	@SuppressWarnings("static-access")
	private void driveForward() {
		if(Robot.timer.get() > futureTime) { // Adding Delay to make sure autoCommand takes effect before checking
			if(chassis.isDone()) {
				autoState = State.STOP;
				chassis.addAutoCommand(factory.command_Stop());
			}
		}
	}
	
	private void stop() {
		chassis.driveAuto();
	}

	@Override
	public String getStateString()
	{
		switch(autoState) {
			case BEGIN:
				return "Begin";
			case DRIVE_FORWARD:
				return "Driving Forward";
			case STOP:
				return "Stopped";
			default:
				return "NULL";			
		}
	}
	
	public void reset() {
		chassis.reset();
		autoState = State.BEGIN;
	}
}
