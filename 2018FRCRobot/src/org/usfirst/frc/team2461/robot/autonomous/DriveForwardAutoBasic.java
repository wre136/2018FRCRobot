package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;

public class DriveForwardAutoBasic implements AutoCode
{
	private SwerveDrive chassis;
	private double timeNow;
	private double timeFuture;
	
	private double timeToDriveForward = 3;
	
	private enum State {
		BEGIN, DRIVE_FORWARD, STOP;
		
		/**
		 * Method to get name of current enum value
		 * Returns String representing the current enum value
		 */
		@Override
		public String toString() {
			String enumName = "";
			
			if(this.ordinal() == 0) {
				enumName = "BEGIN";
			} else if(this.ordinal() == 1) {
				enumName = "DRIVE_FORWARD";
			} else {
				enumName = "STOP";
			}
			
			return enumName;
		}
	}
	
	private State autoState;
	private State autoStatePrevious;
	
	public DriveForwardAutoBasic(SwerveDrive chassisIn) {
		chassis = chassisIn;
	}
	
	@Override
	public void run()
	{
		switch(autoState) {
			case BEGIN:
				begin();
				break;
			case DRIVE_FORWARD:
				driveForward();
				break;
			case STOP:
				break;
			default:
				break;
		}
		
	}
	
	private void begin() {
		setNextState(State.DRIVE_FORWARD);
		chassis.driveManual(180, 1);
		timeFuture = Robot.timer.get() + timeToDriveForward;
	}
	
	private void driveForward() {
		timeNow = Robot.timer.get();
		
		if(timeNow > timeFuture) {
			setNextState(State.STOP);
			chassis.driveManual(0, 0);
		}
	}

	@Override
	public String getStateString()
	{
		return autoState.toString();
	}

	@Override
	public String getStatePreviousString()
	{
		return autoStatePrevious.toString();
	}

	@Override
	public void reset()
	{
		
	}
	
	private void setNextState(State nextState) {
		autoStatePrevious = autoState;
		autoState = nextState;
	}

}
