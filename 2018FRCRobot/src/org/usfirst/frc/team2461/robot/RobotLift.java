package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * <h1> Robot Lifter Class </h1>
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * <p>
 * 2018 season Robot Lifter subsystem that is lift the robot.
 * </p>
 */
public class RobotLift
{
	private SpeedController[] motor = new SpeedController[2];
	private MetalSkinsController player;
	
	private enum State {
		BEGIN, LOWERING, LIFTING, IDLE
	}
	
	private State stateNow;
	
	/**
	 * Creates a RobotLifter object that has a middle switch.
	 * @param motor1 Motor 1
	 * @param motor2 Motor 2
	 * @param playerIn MetalSkinsController to control the robot lift
	 */
	public RobotLift(SpeedController motor1, SpeedController motor2, MetalSkinsController playerIn)
	{
		motor[0] = motor1;
		motor[1] = motor2;
		player = playerIn;
		stateNow = State.BEGIN;
	}
	
	/**
	 * This is the method that runs the Robot Lifter in Teleop.
	 * 
	 * <p>
	 * Call this method in Teleop to run the Robot Lifter using
	 * the State Machine mechanics and controller input.
	 * </p>
	 */
	public void run() {
		switch(stateNow) {
			case BEGIN:
				begin();
				break;
			case IDLE:
				idle();
				break;
			case LIFTING:
				lifting();
				break;
			case LOWERING:
				lowering();
				break;
			default:
				break;
			
		}
	}
	
	/**
	 * Method for the BEGIN State of the Robot Lifter state machine.
	 * <p>
	 * It stops the Robot Lifter motors and sets
	 * the state to IDLE.
	 * </p>
	 */
	private void begin() {
		stopLift();
		stateNow = State.IDLE;
	}
	
	/**
	 * Method for the IDLE State of the Robot Lifter state machine.
	 * <p>
	 * This is the state where the Robot Lifter will not be doing anything.
	 * </p>
	 * <p>
	 * Based on controller inputs:
	 * <ul>
	 * <li>The <b>Y Button</b> will raise the robot and move 
	 * the Robot Lifter state machine to LIFTING </li>
	 * <li>The <b>X Button</b> will lower the robot and move 
	 * the Robot Lifter state machine to LOWERING </li> 
	 * </ul>
	 * </p>
	 */
	private void idle() {
		if(player.getYButton() && player.getXButton()) {
			return;
		} else if(player.getYButton()) {
			liftRobot();
			stateNow = State.LIFTING;
		} else if(player.getXButton()) {
			lowerRobot();
			stateNow = State.LOWERING;
		}
	}
	
	/**
	 * Method for the LIFTING State of the Robot Lifter state machine.
	 * <p>
	 * This is the state where the Robot Lifter will be lifting.
	 * </p>
	 * <p>
	 * Based on controller inputs, letting go of the <b>Y Button</b> will 
	 * stop the lifter and move the Robot Lifter state machine to IDLE.
	 * </p>
	 */
	private void lifting() {
		if(!player.getYButton()) {
			stopLift();
			stateNow = State.IDLE;
		}
	}
	
	/**
	 * Method for the LOWERING State of the Robot Lifter state machine.
	 * <p>
	 * This is the state where the Robot Lifter will be lowering.
	 * </p>
	 * <p>
	 * Based on controller inputs, letting go of the <b>X Button</b> will 
	 * stop the lifter and move the Robot Lifter state machine to IDLE.
	 * </p>
	 */
	private void lowering() {
		if(!player.getXButton()) {
			stopLift();
			stateNow = State.IDLE;
		}
	}
	
	private void setMotor(double value) {
		motor[0].set(value);
		motor[1].set(value);
	}
	
	public State getState() {
		return stateNow;
	}
	
	/**
	 * Returns the current state of the Robot Lifter State Machine as a String object
	 * @return Current state of the Robot Lifter state machine as a String object
	 */
	public String getStateString() {
		switch(stateNow) {
			case BEGIN:
				return "BEGIN";
			case IDLE:
				return "IDLE";
			case LIFTING:
				return "LIFTING";
			case LOWERING:
				return "LOWERING";
			default:
				return "NULL";
		}
	}
	
	/**
	 * Manual method to stop the Robot Lifter.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Robt Lifter State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void stopLift() {
		setMotor(0);
	}
	
	/**
	 * Manual method to lift the Robot Lifter.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Robt Lifter State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void liftRobot() {
		setMotor(-1);
	}
	
	/**
	 * Manual method to lower the Robot Lifter.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Robt Lifter State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void lowerRobot() {
		setMotor(1);
	}
}
