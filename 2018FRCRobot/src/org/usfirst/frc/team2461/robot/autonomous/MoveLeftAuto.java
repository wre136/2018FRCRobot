package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.BoxManager;
import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;
import org.usfirst.frc.team2461.robot.SwerveDriveAutoCommandFactory;

public class MoveLeftAuto implements AutoCode
{
	private enum DrivingState {
		BEGIN, DRIVE_FORWARD, MOVE_LEFT, STOP
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
	private double timeFuture;
	private double timeNow;
	
	private SwerveDriveAutoCommandFactory factory = SwerveDriveAutoCommandFactory.getInstance();
	private double autoSwitchDistance = 60; //set to 60inches for testing purposes
	private double autoMoveLeftDistance = 48;
	private double autoStartRisingBoxDistance = 36;
	private double autoStartSpittingBoxDistance = 58;
	
	private double spitOutTime = 2;
	
	public MoveLeftAuto(SwerveDrive chassisIn, BoxManager boxMaangerIn) {
		chassis = chassisIn;
		boxManager = boxMaangerIn;
		drivingState = DrivingState.BEGIN;
		boxCollectorState = BoxCollectorState.BEGIN;
	}
	
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
	
	@SuppressWarnings("static-access")
	private void driveBegin() {
		chassis.clearAutoCommands();
		chassis.addAutoCommand(factory.command_GoForward(autoSwitchDistance));
		chassis.driveAuto();
		drivingState = DrivingState.DRIVE_FORWARD;
		drivingStatePrevious = DrivingState.BEGIN;
		timeFuture = Robot.timer.get() + 0.1;
	}
	
	@SuppressWarnings("static-access")
	private void driveDriveForward() {
		if(Robot.timer.get() > timeFuture) { // Adding Delay to make sure autoCommand takes effect before checking
			if(!chassis.isDone()) {
				drivingState = DrivingState.MOVE_LEFT;
				drivingStatePrevious = DrivingState.DRIVE_FORWARD;
				chassis.addAutoCommand(factory.command_MoveLeft(autoMoveLeftDistance));
				//chassis.driveAuto(); // Not sure if this will be needed
			}
		}
	}
	
	@SuppressWarnings("static-access")
	private void driveMoveLeft() {
		chassis.driveAuto();

		if(chassis.isDone()) {
			drivingState = DrivingState.STOP;
			chassis.addAutoCommand(factory.command_Stop());
			drivingStatePrevious = DrivingState.MOVE_LEFT;
		}
	}
	
	private void driveStop() {
		chassis.driveAuto();
	}
	
	private void boxBegin() {
		boxManager.boxCollector.armsExtend();
		timeFuture = Robot.timer.get() + 0.5;
		boxCollectorState = BoxCollectorState.EXTENDING_ARMS;
		boxCollectorStatePrevious = BoxCollectorState.BEGIN;
	}
	
	private void boxExtendingArms() {
		timeNow = Robot.timer.get();
		if(timeNow >= timeFuture) {
			boxManager.boxLifter.lower();
			boxCollectorState = BoxCollectorState.LOWERING;
			boxCollectorStatePrevious = BoxCollectorState.EXTENDING_ARMS;
		}
	}
	
	private void boxLowering() {
		if(boxManager.boxLifter.getSwitchLow()) {
			if(boxCollectorStatePrevious == BoxCollectorState.SPIITING_OUT) {
				boxManager.stopBoxSucker();
				boxCollectorState = BoxCollectorState.DONE;
				boxCollectorStatePrevious = BoxCollectorState.LOWERING;
			} else {
				boxManager.boxLifter.stop();
				boxCollectorState = BoxCollectorState.IDLE;
				boxCollectorStatePrevious = BoxCollectorState.LOWERING;
			}
		}
	}
	
	private void boxRising() {
		if(boxManager.boxLifter.getSwitchMiddle()) {
			boxManager.boxLifter.stop();
			boxCollectorState = BoxCollectorState.IDLE;
			boxCollectorStatePrevious = BoxCollectorState.RISING;
		}
	}
	
	private void boxSpittingOut() {
		timeNow = Robot.timer.get();
		
		if(timeNow >= timeFuture) {
			boxManager.stopBoxSucker();
			boxManager.boxLifter.lower();
			boxCollectorState = BoxCollectorState.LOWERING;
			boxCollectorStatePrevious = BoxCollectorState.SPIITING_OUT;
		}
	}
	
	private void boxIdle() {
		if(chassis.getDistanceAvg() >= autoStartSpittingBoxDistance) {
			boxManager.spitBoxOut();
			timeFuture = Robot.timer.get() + spitOutTime;
			boxCollectorState = BoxCollectorState.SPIITING_OUT;
			boxCollectorStatePrevious = BoxCollectorState.IDLE;
		} else if(chassis.getDistanceAvg() >= autoStartRisingBoxDistance) {
			if(!boxManager.boxLifter.getSwitchMiddle()) {
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
			default:
				return "NULL";
		}
	}

	@Override
	public void reset()
	{
		chassis.reset();
		chassis.clearAutoCommands();
		drivingState = DrivingState.BEGIN;
		boxCollectorState = BoxCollectorState.BEGIN;
	}

	
}
