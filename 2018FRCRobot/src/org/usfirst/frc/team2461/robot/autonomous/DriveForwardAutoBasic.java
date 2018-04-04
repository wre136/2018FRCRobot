package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.BoxManager;
import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;

public class DriveForwardAutoBasic implements AutoCode
{
	private SwerveDrive chassis;
	private BoxManager boxManager;
	private double timeNow;
	private double timeFuture;
	
	/**
	 * Double value to hold how many seconds to drive the robot forward
	 */
	private double timeToDriveForward = 1.5;
	
	private enum State {
		BEGIN, DRIVE_FORWARD, STOP;
		
		/**
		 * Method to get name of current enum value
		 * Returns String representing the current enum value
		 */
		@Override
		public String toString() {
			switch(this) {
				case BEGIN:
					return "BEGIN";
				case DRIVE_FORWARD:
					return "DRIVE_FORWARD";
				case STOP:
					return "STOP";
				default:
					return "NULL";
			}
		}
	}
	
	private State autoState;
	private State autoStatePrevious;
	
	public DriveForwardAutoBasic(SwerveDrive chassisIn, BoxManager boxManagerIn) {
		chassis = chassisIn;
		boxManager = boxManagerIn;
		autoState = State.BEGIN;
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
		boxManager.boxCollectorArmRetract();
		chassis.driveManual(0, 1);
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
		setNextState(State.BEGIN);
	}
	
	private void setNextState(State nextState) {
		autoStatePrevious = autoState;
		autoState = nextState;
	}

}
