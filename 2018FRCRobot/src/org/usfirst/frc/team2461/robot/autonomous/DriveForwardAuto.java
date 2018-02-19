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
	private State autoStatePrevious;
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
	
	/**
	 * Method for the BEGIN State of the Drive Forward Auto Code state machine.
	 * <p>
	 * It clears the drive trains list of autoCommands, adds a new command to 
	 * drive forward the distance to the auto line, starts the wheel PID loops, 
	 * moved the autoState to DRIVE_FORWARD, sets the previous state to BEGIN 
	 * and lastly sets future time to the current time plus 0.1 seconds
	 * </p>
	 */
	@SuppressWarnings("static-access")
	private void begin() {
		chassis.clearAutoCommands();
		chassis.addAutoCommand(factory.command_GoForward(autoLineDistance));
		chassis.driveAuto();
		autoState = State.DRIVE_FORWARD;
		autoStatePrevious = State.BEGIN;
		futureTime = Robot.timer.get() + 0.1;
	}
	
	/**
	 * Method for the DRIVE_FORWARD State of the Drive Forward Auto Code state machine.
	 * <p>
	 * Once future time has passed, it will check to see if all wheels have traveled
	 * the distance they needed to and have stopped. If they have, move the state 
	 * to STOP, change previous state to DRIVE_FORWARD and add a new command to stop
	 * to the drive train.
	 * </p>
	 */
	@SuppressWarnings("static-access")
	private void driveForward() {
		if(Robot.timer.get() > futureTime) { // Adding Delay to make sure autoCommand takes effect before checking
			if(!chassis.isDone()) {
				autoState = State.STOP;
				autoStatePrevious = State.DRIVE_FORWARD;
				chassis.addAutoCommand(factory.command_Stop());
			}
		}
	}
	
	/**
	 * Method for the STOP State of the Drive Forward Auto Code state machine.
	 * <p>
	 * It will run the stop command and do nothing else
	 * </p>
	 */
	private void stop() {
		chassis.driveAuto();
	}

	/**
	 * Returns the current state of the Drive Forward Auto Code State Machine as a String object
	 * @return String object representing the Current state of the Drive Forward Auto Code
	 */
	@Override
	public String getStateString()
	{
		switch(autoState) {
			case BEGIN:
				return "BEGIN";
			case DRIVE_FORWARD:
				return "DRIVING_FORWARD";
			case STOP:
				return "STOPPED";
			default:
				return "NULL";			
		}
	}
	
	/**
	 * Returns the previous state of the Drive Forward Auto Code State Machine as a String object
	 * @return String object representing the Current state of the Drive Forward Auto Code
	 */
	@Override
	public String getStatePreviousString()
	{
		switch(autoStatePrevious) {
			case BEGIN:
				return "BEGIN";
			case DRIVE_FORWARD:
				return "DRIVING_FORWARD";
			case STOP:
				return "STOPPED";
			default:
				return "NULL";			
		}
	}
	
	/**
	 * Disables all PID Loops of the drive train, clears all autoCommands 
	 * and sets the Drive Forward Auto Code state to BEGIN
	 */
	public void reset() {
		chassis.reset();
		chassis.clearAutoCommands();
		autoState = State.BEGIN;
	}
}
