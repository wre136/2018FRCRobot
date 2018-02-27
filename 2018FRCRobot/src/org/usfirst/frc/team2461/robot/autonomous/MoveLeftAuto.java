package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.BoxManager;
import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;
import org.usfirst.frc.team2461.robot.SwerveDriveAutoCommandFactory;

public class MoveLeftAuto implements AutoCode
{
	private enum DrivingState {
		BEGIN, DRIVE_FORWARD, MOVE_LEFT, STOP, DRIVE_BACK
	}
	
	private DrivingState drivingState;
	private DrivingState drivingStatePrevious;
	
	private enum BoxCollectorState {
		BEGIN, EXTENDING_ARMS, LOWERING, RISING, SPIITING_OUT, IDLE, DONE
	}
	
	private BoxCollectorState boxCollectorState;
	private BoxCollectorState boxCollectorStatePrevious;
	
	private SwerveDrive chassis;
	private BoxManager boxManager;
	
	//Time Management Elements
	private double timeDriveFuture;
	private double timeDriveNow;
	private double timeBoxManagerFuture;
	private double timeBoxManagerNow;
	
	private SwerveDriveAutoCommandFactory factory = SwerveDriveAutoCommandFactory.getInstance();
	private double autoSwitchDistance = 120; //set to 60inches for testing purposes
	private double autoMoveLeftDistance = 48;
	private double autoDriveBackDistance = 10;
	private double autoStartRisingBoxDistance = 100;
	private double autoStartSpittingBoxDistance = 120;
	
	private double spitOutTime = 3;
	
	public MoveLeftAuto(SwerveDrive chassisIn, BoxManager boxMaangerIn) {
		chassis = chassisIn;
		boxManager = boxMaangerIn;
		drivingState = DrivingState.BEGIN;
		boxCollectorState = BoxCollectorState.BEGIN;
	}
	
	/**
	 * This is the method that runs the MoveLeftAuto code in Autonomous.
	 * 
	 * <p>
	 * Call this method in Autonomous Periodic to run the MoveLeftAuto code using
	 * the State Machine mechanics and sensor input.
	 * </p>
	 */
	public void run() {
		switch(drivingState) {
			case BEGIN:
				driveBegin();
				break;
			case DRIVE_FORWARD:
				driveDriveForward();
				break;
			case MOVE_LEFT:
				driveMoveLeft();
				break;
			case STOP:
				driveStop();
				break;
			case DRIVE_BACK:
				driveDriveBack();
				break;
			default:
				break;
			
		}
		
		switch(boxCollectorState) {
			case BEGIN:
				boxBegin();
				break;
			case EXTENDING_ARMS:
				boxExtendingArms();
				break;
			case IDLE:
				boxIdle();
				break;
			case LOWERING:
				boxLowering();
				break;
			case RISING:
				boxRising();
				break;
			case SPIITING_OUT:
				boxSpittingOut();
				break;
			case DONE:
				break;
			default:
				break;
			
		}
	}
	
	/**
	 * Method for the BEGIN State of the driving state machine.
	 * <p><ol>
	 * <li>Clears the drive train of all autoCommands</li>
	 * <li>Adds an autoCommand to drive forward the autoSwitchDistance distance</li>
	 * <li>Activates the turning and driving PID Loops</li>
	 * <li>Changes the DrivingState to the DRIVE_FORWARD state while changing the DrivingStatePrevious state to
	 * BEGIN</li>
	 * <li>Creates a time variable that is 0.1 seconds from now to verify
	 * that we do not check the drive train for being finished just yet</li>
	 * </ol></p>
	 */
	@SuppressWarnings("static-access")
	private void driveBegin() {
		chassis.clearAutoCommands();
		chassis.addAutoCommand(factory.command_GoForward(autoSwitchDistance));
		chassis.driveAuto();
		drivingState = DrivingState.DRIVE_FORWARD;
		drivingStatePrevious = DrivingState.BEGIN;
		timeDriveFuture = Robot.timer.get() + 0.1;
	}
	
	/**
	 * Method for the DRIVE_FORWARD State of the driving state machine.
	 * <p><ol>
	 * <li>Gets the current time</li>
	 * <li>Checks to see if current time is greater than future driving time</li>
	 * <li>If so, evaluate if drive train has moved as needed</li>
	 * <li>If so, change the DrivingState to the MOVE_LEFT state while changing the DrivingStatePrevious state to
	 * DRIVE_FORWARD</li>
	 * <li>Reset the drive and turn PID Loops of the drive train</li>
	 * <li>Add a new AutoCommand to move the robot left the specified distance</li>
	 * <li>Restart the drive and turn PID loops</li>
	 * <li>Creates a time variable that is 0.1 seconds from now to verify
	 * that we do not check the drive train for being finished just yet</li>
	 * </ol></p>
	 */
	@SuppressWarnings("static-access")
	private void driveDriveForward() {
		timeDriveNow = Robot.timer.get();
		if(timeDriveNow > timeDriveFuture) { // Adding Delay to make sure autoCommand takes effect before checking
			if(!chassis.isDone()) {
				drivingState = DrivingState.MOVE_LEFT;
				drivingStatePrevious = DrivingState.DRIVE_FORWARD;
				chassis.reset();
				chassis.addAutoCommand(factory.command_MoveLeft(autoMoveLeftDistance));
				chassis.driveAuto(); // Not sure if this will be needed
				timeDriveFuture = Robot.timer.get() + 0.1;
			}
		}
	}

	/**
	 * Method for the MOVE_LEFT State of the driving state machine.
	 * <p><ol>
	 * <li>Gets the current time</li>
	 * <li>Checks to see if current time is greater than future driving time</li>
	 * <li>If so, evaluate if drive train has moved as needed</li>
	 * <li>If so, change the DrivingState to the STOP state while changing the DrivingStatePrevious state to
	 * MOVE_LEFT</li>
	 * <li>Reset the drive and turn PID Loops of the drive train</li>
	 * <li>Add a new AutoCommand to stop the robot</li>
	 * <li>Restart the drive and turn PID loops</li>
	 * <li>Creates a time variable that is 0.1 seconds from now to verify
	 * that we do not check the drive train for being finished just yet</li>
	 * </ol></p>
	 */
	@SuppressWarnings("static-access")
	private void driveMoveLeft() {
		timeDriveNow = Robot.timer.get();
		if(timeDriveNow > timeDriveFuture) {
			if(!chassis.isDone()) {
				drivingState = DrivingState.STOP;
				drivingStatePrevious = DrivingState.MOVE_LEFT;
				chassis.reset();
				chassis.addAutoCommand(factory.command_Stop());
				chassis.driveAuto();
			}
		}
	}
	
	/**
	 * Method for the DRIVE_BACK State of the driving state machine.
	 * <p><ol>
	 * <li>Gets the current time</li>
	 * <li>Checks to see if current time is greater than future driving time</li>
	 * <li>If so, evaluate if drive train has moved as needed</li>
	 * <li>If so, change the DrivingState to the STOP state while changing the DrivingStatePrevious state to
	 * DRIVE_BACK</li>
	 * <li>Reset the drive and turn PID Loops of the drive train</li>
	 * <li>Add a new AutoCommand to stop the robot</li>
	 * <li>Restart the drive and turn PID loops</li>
	 * <li>Extend the arms of the Box Collector</li>
	 * <li>Lower the Box Lifter to the low position</li>
	 * <li>If so, change the BoxCollectorState to the LOWERING state while changing the BoxCollectorStatePrevious state to
	 * DONE</li>
	 * </ol></p>
	 */
	@SuppressWarnings("static-access")
	private void driveDriveBack() {
		timeDriveNow = Robot.timer.get();
		if(timeDriveNow > timeDriveFuture) {
			if(!chassis.isDone()) {
				drivingState = DrivingState.STOP;
				drivingStatePrevious = DrivingState.DRIVE_BACK;
				chassis.reset();
				chassis.addAutoCommand(factory.command_Stop());
				chassis.driveAuto();
				
				//Once the robot has driven back, extend arms and lower the Box Lifter
				boxManager.boxCollector.armsExtend();
				boxManager.boxLifter.lower();
				boxCollectorState = BoxCollectorState.LOWERING;
				boxCollectorStatePrevious = BoxCollectorState.DONE;
			}
		}
	}
	
	/**
	 * Method for the STOP State of the driving state machine.
	 * <p>Verify that the robot gets to a stop state</p>
	 */
	private void driveStop() {
		chassis.driveAuto();
	}
	
	/**
	 * Method for the BEGIN State of the Box Collector state machine.
	 * <p><ol>
	 * <li>Extends the arms of the Box Collector</li>
	 * <li>Creates a time variable that is 0.5 seconds from now to verify
	 * that we do not lower the box lifter until the arms have had time to
	 * open</li>
	 * <li>Changes the BoxCollectorState to the EXTENDING_ARMS state while changing the BoxCollectorStatePrevious state to
	 * BEGIN</li>
	 * </ol></p>
	 */
	private void boxBegin() {
		boxManager.boxCollector.armsExtend();
		timeBoxManagerFuture = Robot.timer.get() + 0.5;
		boxCollectorState = BoxCollectorState.EXTENDING_ARMS;
		boxCollectorStatePrevious = BoxCollectorState.BEGIN;
	}
	
	/**
	 * Method for the EXTENDING_ARMS State of the Box Collector state machine.
	 * <p><ol>
	 * <li>Gets the current time</li>
	 * <li>Checks if current time is greater that futureBoxManager time</li>
	 * <li>If so, lower the Box Lifter to the low position</li>
	 * <li>Changes the BoxCollectorState to the LOWERING state while changing the BoxCollectorStatePrevious state to
	 * EXTENDING_ARMS</li>
	 * </ol></p>
	 */
	private void boxExtendingArms() {
		timeBoxManagerNow = Robot.timer.get();
		if(timeBoxManagerNow >= timeBoxManagerFuture) {
			boxManager.boxLifter.lower();
			boxCollectorState = BoxCollectorState.LOWERING;
			boxCollectorStatePrevious = BoxCollectorState.EXTENDING_ARMS;
		}
	}
	
	/**
	 * Method for the LOWERING State of the Box Collector state machine.
	 * <p><ol>
	 * <li>Gets if the lower position switch is hit</li>
	 * <li>If so, checks if the boxCollectorStatePrevious is DONE</li>
	 * <ol><li>If so, stop the Box Lifter</li>
	 * <li>Changes the BoxCollectorState to the DONE state while changing the BoxCollectorStatePrevious state to
	 * LOWERING</li></ol>
	 * <li>Else</li> <ol>
	 * <li>Stop the Box Lifter</li>
	 * <li>Changes the BoxCollectorState to the IDLE state while changing the BoxCollectorStatePrevious state to
	 * LOWERING</li></ol>
	 * </ol></p>
	 */
	private void boxLowering() {
		if(boxManager.boxLifter.getSwitchLow()) {
			if(boxCollectorStatePrevious == BoxCollectorState.DONE) {
				boxManager.boxLifter.stop();
				boxCollectorState = BoxCollectorState.DONE;
				boxCollectorStatePrevious = BoxCollectorState.LOWERING;
			} else {
				boxManager.boxLifter.stop();
				boxCollectorState = BoxCollectorState.IDLE;
				boxCollectorStatePrevious = BoxCollectorState.LOWERING;
			}
		}
	}
	
	/**
	 * Method for the RISING State of the Box Collector state machine.
	 * <p><ol>
	 * <li>Gets if the middle position switch is hit</li>
	 * <li>If so, stop the Box Lifter</li>
	 * <li>Retract the Box Collector Arms</li>
	 * <li>Changes the BoxCollectorState to the IDLE state while changing the BoxCollectorStatePrevious state to
	 * RISING</li>
	 * </ol></p>
	 */
	private void boxRising() {
		if(boxManager.boxLifter.getSwitchMiddle()) { // Once we reach the middle switch
			boxManager.boxLifter.stop();
			boxManager.boxCollector.armsRetract();
			boxCollectorState = BoxCollectorState.IDLE;
			boxCollectorStatePrevious = BoxCollectorState.RISING;
		}
	}
	
	/**
	 * Method for the SPITTING_OUT State of the Box Collector state machine.
	 * <p><ol>
	 * <li>Gets the current time and sees if its greater than future BoxManager Time</li>
	 * <li>If so, stop the Box Sucker</li>
	 * <li>Changes the BoxCollectorState to the DONE state while changing the BoxCollectorStatePrevious state to
	 * SPITTING_OUT</li>
	 * <li>Changes the DrivingState to the DRIVE_BACK state while changing the BoxCollectorStatePrevious state to
	 * STOP</li>
	 * <li>Disable the PID Loops of the Drive Train and clear its autoCommands</li>
	 * <li>Add autoCommand for robot to drive back the specified distance</li>
	 * <li>Start the drive train PID Loops</li>
	 * <li>Create timer object to prevent drive train from being checked until later</li>
	 * </ol></p>
	 */
	@SuppressWarnings("static-access")
	private void boxSpittingOut() {
		timeBoxManagerNow = Robot.timer.get();
		if(timeBoxManagerNow >= timeBoxManagerFuture) {
			boxManager.stopBoxSucker();
			//boxManager.boxLifter.lower(); //Not lowering Box Collector to protect the arms
			//boxCollectorState = BoxCollectorState.LOWERING;
			boxCollectorState = BoxCollectorState.DONE;
			boxCollectorStatePrevious = BoxCollectorState.SPIITING_OUT;
			
			//Once spitting out is done, drive robot back 10 inches
			drivingState = DrivingState.DRIVE_BACK;
			drivingStatePrevious = DrivingState.STOP;
			chassis.reset();
			chassis.addAutoCommand(factory.command_GoBackward(autoDriveBackDistance));
			chassis.driveAuto();
			timeDriveFuture = Robot.timer.get() + 0.1;
		}
	}
	
	private void boxIdle() {
		if(chassis.getDistanceAvg() >= autoStartSpittingBoxDistance) {
			boxManager.spitBoxOut();
			timeBoxManagerFuture = Robot.timer.get() + spitOutTime;
			boxCollectorState = BoxCollectorState.SPIITING_OUT;
			boxCollectorStatePrevious = BoxCollectorState.IDLE;
		} else if(chassis.getDistanceAvg() >= autoStartRisingBoxDistance) {
			if(!boxManager.boxLifter.getSwitchMiddle()) { //If we haven't reached the middle switch let
				boxManager.boxLifter.rise();
				boxCollectorState = BoxCollectorState.RISING;
				boxCollectorStatePrevious = BoxCollectorState.IDLE;
			}
		}
	}

	@Override
	public String getStateString()
	{
		return drivingStateToString() + " " + boxCollectorStateToString();
	}
	
	@Override
	public String getStatePreviousString()
	{
		return drivingStatePreviousToString() + " " + boxCollectorStatePreviousToString();
	}
	
	private String drivingStateToString() {
		switch(drivingState) {
			case BEGIN:
				return "Driving State: Begin";
			case DRIVE_FORWARD:
				return "Driving State: Driving Forward";
			case MOVE_LEFT:
				return "Driving State: Moving Left";
			case STOP:
				return "Driving State: Stopped";
			case DRIVE_BACK:
				return "Driving State: Driving Back";
			default:
				return "NULL";
		}
	}
	
	private String drivingStatePreviousToString() {
		switch(drivingStatePrevious) {
			case BEGIN:
				return "Driving State: Begin";
			case DRIVE_FORWARD:
				return "Driving State: Driving Forward";
			case MOVE_LEFT:
				return "Driving State: Moving Left";
			case STOP:
				return "Driving State: Stopped";
			case DRIVE_BACK:
				return "Driving State: Driving Back";
			default:
				return "NULL";
		}
	}
	
	private String boxCollectorStateToString() {
		switch(boxCollectorState) {
			case BEGIN:
				return "Box Collector State: Begin";
			case EXTENDING_ARMS:
				return "Box Collector State: Extending Arms";
			case IDLE:
				return "Box Collector State: Idle";
			case LOWERING:
				return "Box Collector State: Lowering";
			case RISING:
				return "Box Collector State: Rising";
			case SPIITING_OUT:
				return "Box Collector State: Spitting Out";
			case DONE:
				return "BoxCollector State: Done";
			default:
				return "NULL";
		}
	}
	
	private String boxCollectorStatePreviousToString() {
		switch(boxCollectorStatePrevious) {
			case BEGIN:
				return "Box Collector State: Begin";
			case EXTENDING_ARMS:
				return "Box Collector State: Extending Arms";
			case IDLE:
				return "Box Collector State: Idle";
			case LOWERING:
				return "Box Collector State: Lowering";
			case RISING:
				return "Box Collector State: Rising";
			case SPIITING_OUT:
				return "Box Collector State: Spitting Out";
			case DONE:
				return "BoxCollector State: Done";
			default:
				return "NULL";
		}
	}

	/**
	 * Method to disable the Drive and Turn PID Loops of the drive train
	 * , clears all autoCommands and sets the DrivingState and BoxCollectorState
	 * state machines back to their BEGIN states
	 */
	@Override
	public void reset()
	{
		chassis.reset();
		chassis.clearAutoCommands();
		drivingState = DrivingState.BEGIN;
		boxCollectorState = BoxCollectorState.BEGIN;
	}
}
