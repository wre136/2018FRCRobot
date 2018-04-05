package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;
import org.usfirst.frc.team2461.robot.SwerveDriveAutoCommandFactory;

public class DriveForwardAuto implements AutoCode
{
	private enum State {
		BEGIN, DRIVE_FORWARD, DRIVE_LEFT, DRIVE_TO_LINE, STOP
	}
	
	private State autoState;
	private State autoStatePrevious;
	private SwerveDrive chassis;
	private SwerveDriveAutoCommandFactory factory = SwerveDriveAutoCommandFactory.getInstance();
	
	private double distanceDriveForward = 24; //set to 60inches for testing purposes
	private double distanceDriveToSide = 60;
	private double autoLineDistance = 120; //set to 60inches for testing purposes
	private double timeFuture;
	
	public DriveForwardAuto(SwerveDrive driveTrain)
	{
		chassis = driveTrain;
		autoState = State.BEGIN;
		autoStatePrevious = State.BEGIN;
	}
	
	public void run() {
		switch(autoState) {
			case BEGIN:
				begin();
				break;
			case DRIVE_FORWARD:
				driveForward();
				break;
			case DRIVE_LEFT:
				driveLeft();
				break;
			case DRIVE_TO_LINE:
				driveToLine();
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
		chassis.addAutoCommand(factory.command_GoForward(distanceDriveForward));
		chassis.driveAuto();
		setNextState(State.DRIVE_FORWARD);
		timeFuture = Robot.timer.get() + 0.1;
	}
	
	/**
	 * Method for the DRIVE_FORWARD State of the Drive Forward Auto Code state machine.
	 * <p>
	 * Once future time has passed, it will check to see if all wheels have traveled
	 * the distance they needed to and have stopped. If they have, move the state 
	 * to DRIVE_LEFT, change previous state to DRIVE_FORWARD and add a new command to stop
	 * to the drive train.
	 * </p>
	 */
	@SuppressWarnings("static-access")
	private void driveForward() {
		if(Robot.timer.get() > timeFuture) { // Adding Delay to make sure autoCommand takes effect before checking
			if(!chassis.isDone()) {
				setNextState(State.DRIVE_LEFT);
				chassis.addAutoCommand(factory.command_MoveLeft(distanceDriveToSide));
				chassis.driveAuto();
				timeFuture = Robot.timer.get() + 0.1;
			}
		}
	}
	
	/**
	 * Method for the DRIVE_LEFT State of the Drive Forward Auto Code state machine.
	 * <p>
	 * Once future time has passed, it will check to see if all wheels have traveled
	 * the distance they needed to and have stopped. If they have, move the state 
	 * to DRIVE_TO_LINE, change previous state to DRIVE_LEFT and add a new command to stop
	 * to the drive train.
	 * </p>
	 */
	@SuppressWarnings("static-access")
	private void driveLeft() {
		if(Robot.timer.get() > timeFuture) { // Adding Delay to make sure autoCommand takes effect before checking
			if(!chassis.isDone()) {
				setNextState(State.DRIVE_TO_LINE);
				chassis.addAutoCommand(factory.command_GoForward(autoLineDistance));
				chassis.driveAuto();
				timeFuture = Robot.timer.get() + 0.1;
			}
		}
	}
	
	/**
	 * Method for the DRIVE_TO_LINE State of the Drive Forward Auto Code state machine.
	 * <p>
	 * Once future time has passed, it will check to see if all wheels have traveled
	 * the distance they needed to and have stopped. If they have, move the state 
	 * to STOP, change previous state to DRIVE_TO_LINE and add a new command to stop
	 * to the drive train.
	 * </p>
	 */
	@SuppressWarnings("static-access")
	private void driveToLine() {
		if(Robot.timer.get() > timeFuture) { // Adding Delay to make sure autoCommand takes effect before checking
			if(!chassis.isDone()) {
				setNextState(State.STOP);
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
		return autoState.name();
	}
	
	/**
	 * Returns the previous state of the Drive Forward Auto Code State Machine as a String object
	 * @return String object representing the Current state of the Drive Forward Auto Code
	 */
	@Override
	public String getStatePreviousString()
	{
		return autoStatePrevious.name();
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
	
	private void setNextState(State nextState) {
		autoStatePrevious = autoState;
		autoState = nextState;
	}
}
