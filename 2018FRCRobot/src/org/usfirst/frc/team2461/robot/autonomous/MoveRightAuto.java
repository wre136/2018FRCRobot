package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.BoxManager;
import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;
import org.usfirst.frc.team2461.robot.SwerveDriveAutoCommandFactory;

public class MoveRightAuto implements AutoCode
{
	private enum DrivingState {
		BEGIN, DRIVE_FORWARD, MOVE_RIGHT, STOP, DRIVE_BACK
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
	
	public MoveRightAuto(SwerveDrive chassisIn, BoxManager boxMaangerIn) {
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
			case MOVE_RIGHT:
				driveMoveRight();
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
	
	@SuppressWarnings("static-access")
	private void driveBegin() {
		chassis.clearAutoCommands();
		chassis.addAutoCommand(factory.command_GoForward(autoSwitchDistance));
		chassis.driveAuto();
		drivingState = DrivingState.DRIVE_FORWARD;
		drivingStatePrevious = DrivingState.BEGIN;
		timeDriveFuture = Robot.timer.get() + 0.1;
	}
	
	@SuppressWarnings("static-access")
	private void driveDriveForward() {
		timeDriveNow = Robot.timer.get();
		if(timeDriveNow > timeDriveFuture) { // Adding Delay to make sure autoCommand takes effect before checking
			if(!chassis.isDone()) {
				drivingState = DrivingState.MOVE_RIGHT;
				drivingStatePrevious = DrivingState.DRIVE_FORWARD;
				chassis.reset();
				chassis.addAutoCommand(factory.command_MoveRight(autoMoveLeftDistance));
				chassis.driveAuto(); // Not sure if this will be needed
				timeDriveFuture = Robot.timer.get() + 0.1;
			}
		}
	}

	@SuppressWarnings("static-access")
	private void driveMoveRight() {
		timeDriveNow = Robot.timer.get();
		if(timeDriveNow > timeDriveFuture) {
			if(!chassis.isDone()) {
				drivingState = DrivingState.STOP;
				drivingStatePrevious = DrivingState.MOVE_RIGHT;
				chassis.reset();
				chassis.addAutoCommand(factory.command_Stop());
				chassis.driveAuto();
			}
		}
	}
	
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
	
	private void driveStop() {
		chassis.driveAuto();
	}
	
	private void boxBegin() {
		boxManager.boxCollector.armsExtend();
		timeBoxManagerFuture = Robot.timer.get() + 0.5;
		boxCollectorState = BoxCollectorState.EXTENDING_ARMS;
		boxCollectorStatePrevious = BoxCollectorState.BEGIN;
	}
	
	private void boxExtendingArms() {
		timeBoxManagerNow = Robot.timer.get();
		if(timeBoxManagerNow >= timeBoxManagerFuture) {
			boxManager.boxLifter.lower();
			boxCollectorState = BoxCollectorState.LOWERING;
			boxCollectorStatePrevious = BoxCollectorState.EXTENDING_ARMS;
		}
	}
	
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
	
	private void boxRising() {
		if(boxManager.boxLifter.getSwitchMiddle()) { // Once we reach the middle switch
			boxManager.boxLifter.stop();
			boxManager.boxCollector.armsRetract();
			boxCollectorState = BoxCollectorState.IDLE;
			boxCollectorStatePrevious = BoxCollectorState.RISING;
		}
	}
	
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
			case MOVE_RIGHT:
				return "Driving State: Moving RIGHT";
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
			case MOVE_RIGHT:
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

	@Override
	public void reset()
	{
		chassis.reset();
		chassis.clearAutoCommands();
		drivingState = DrivingState.BEGIN;
		boxCollectorState = BoxCollectorState.BEGIN;
	}
}
