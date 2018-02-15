package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.BoxCollector;
import org.usfirst.frc.team2461.robot.BoxLifter;
import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.SwerveDrive;
import org.usfirst.frc.team2461.robot.SwerveDriveAutoCommandFactory;

public class MoveRightAuto implements AutoCode
{
	private enum DrivingState {
		BEGIN, DRIVE_FORWARD, MOVE_RIGHT, STOP
	}
	
	private DrivingState drivingState;
	
	private enum BoxCollectorState {
		BEGIN, EXTENDING_ARMS, LOWERING, RISING, SPIITING_OUT, IDLE
	}
	
	private BoxCollectorState boxCollectorState;
	
	private SwerveDrive chassis;
	private BoxCollector boxCollector;
	private BoxLifter boxLifter;
	private double timeFuture;
	private double timeNow;
	
	private SwerveDriveAutoCommandFactory factory = SwerveDriveAutoCommandFactory.getInstance();
	private double autoSwitchDistance = 60; //set to 60inches for testing purposes
	private double autoMoveRightDistance = 48;
	
	private double spitOutTime = 2;
	
	public MoveRightAuto(SwerveDrive chassisIn, BoxCollector boxCollectorIn, BoxLifter boxLifterIn) {
		chassis = chassisIn;
		boxCollector = boxCollectorIn;
		boxLifter = boxLifterIn;
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
			default:
				break;
			
		}
	}
	
	@SuppressWarnings("static-access")
	private void driveBegin() {
		chassis.addAutoCommand(factory.command_GoForward(autoSwitchDistance));
	}
	
	@SuppressWarnings("static-access")
	private void driveDriveForward() {
		chassis.driveAuto();

		if(chassis.isDone()) {
			drivingState = DrivingState.MOVE_RIGHT;
			boxCollectorState = BoxCollectorState.RISING;
			chassis.addAutoCommand(factory.command_MoveRight(autoMoveRightDistance));
		}
	}
	
	@SuppressWarnings("static-access")
	private void driveMoveRight() {
		chassis.driveAuto();

		if(chassis.isDone()) {
			drivingState = DrivingState.STOP;
			chassis.addAutoCommand(factory.command_Stop());
		}
	}
	
	private void driveStop() {
		chassis.driveAuto();
	}
	
	private void boxBegin() {
		boxCollector.armsExtend();
		timeFuture = Robot.timer.get() + 0.5;
		boxCollectorState = BoxCollectorState.EXTENDING_ARMS;
	}
	
	private void boxExtendingArms() {
		timeNow = Robot.timer.get();
		if(timeNow >= timeFuture) {
			boxLifter.lower();
			boxCollectorState = BoxCollectorState.LOWERING;
		}
	}
	
	private void boxLowering() {
		if(boxLifter.getSwitchLow()) {
			boxLifter.stop();
			boxCollectorState = BoxCollectorState.IDLE;
		}
	}
	
	private void boxRising() {
		boxLifter.rise();
		
		if(boxLifter.getSwitchMiddle()) {
			boxLifter.stop();
			boxCollector.spitBoxOut();
			timeFuture = Robot.timer.get() + spitOutTime;
			boxCollectorState = BoxCollectorState.SPIITING_OUT;
		}
	}
	
	private void boxSpittingOut() {
		timeNow = Robot.timer.get();
		
		if(timeNow >= timeFuture) {
			boxCollector.stopBoxSucker();
			boxLifter.lower();
			boxCollectorState = BoxCollectorState.LOWERING;
		}
	}
	
	private void boxIdle() {
		
	}
}
