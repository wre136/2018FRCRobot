package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.BoxManager;
import org.usfirst.frc.team2461.robot.SwerveDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <h1> MoveLeftAuto Class</h1>
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * <p>
 * 2018 season MoveLeftAuto class that controls the drive train, 
 * the Box Collector and Box Lifter during Autonomous Periodic. It
 * extends MoveAuto class by filling in the abstract methods prepareMoveSideCommand()
 * and printMoveSideState() to indicate the robot will move left
 * </p>
 */
public class MoveLeftAuto extends MoveAuto
{

	/**
	 * Creates a MoveLeftAuto object that controls the drive train, Box Collector 
	 * and Box Lifter during autonomous periodic
	 * @param chassisIn SwerveDrive object representing the drive train
	 * @param boxMaangerIn BoxManager object
	 */
	public MoveLeftAuto(SwerveDrive chassisIn, BoxManager boxMaangerIn) {
		super(chassisIn, boxMaangerIn);
	}

	/**
	 * Used the in the driveDriveForward() method of the parent class, 
	 * this overriding method added a MoveLeft autoCommand to the drive
	 * train
	 */
	@SuppressWarnings("static-access")
	@Override
	protected void prepareMoveSideCommand() {
		chassis.addAutoCommand(factory.command_MoveLeft(autoMoveSideDistance));
		
	}

	/**
	 * Used in the drivingStateToString() method of the parent class, 
	 * it returns the string for when the drive train is moving left.
	 */
	@Override
	protected String printMoveSideState() {
		return "Driving State: Moving Left";
	}
	
	public void debug() {
		SmartDashboard.putString("MoveLeftAuto State", getStateString());
		SmartDashboard.putString("MoveLeftAuto State Previous", getStatePreviousString());
	}
}
