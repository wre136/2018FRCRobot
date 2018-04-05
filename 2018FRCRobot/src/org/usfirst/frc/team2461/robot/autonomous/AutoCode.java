package org.usfirst.frc.team2461.robot.autonomous;

public interface AutoCode
{
	/**
	 * This is the method that runs the autonomous code in Autonomous.
	 * 
	 * <p>
	 * Call this method in Autonomous Periodic to run the autonomous code using
	 * the State Machine mechanics and sensor input.
	 * </p>
	 */
	public void run();
	
	/**
	 * Returns String object that describes what the current state of the autonomous code is. 
	 */
	public String getStateString();
	
	/**
	 * Returns String object that describes what the previous state of the autonomous code was. 
	 */
	public String getStatePreviousString();
	
	/**
	 * Method to disable the Drive and Turn PID Loops of the drive train
	 * , clears all autoCommands and sets any subsystems back to their BEGIN 
	 * states
	 */
	public void reset();
	
	/**
	 * Prints the current state of the Autonomous Code and its previous
	 * state to the SmartDashboard
	 */
	public void debug();
}
