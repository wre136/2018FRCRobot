package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * <h1> Box Lifter Class </h1>
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * <p>
 * 2018 season Box Lifter subsystem that is used to raise and lower the box Collector
 * </p>
 */
public class BoxLifter
{
	private SpeedController motorLifter;
	private DigitalInput switchLower;
	private DigitalInput switchMiddle;
	private DigitalInput switchHigher;
	private MetalSkinsController player;
	
	private enum State {
		BEGIN, IDLE, LOW, MIDDLE, HIGH, LIFTING, LOWERING
	}
	
	private State stateNow;
	private State statePrevious;
	
	private enum Mode {
		AUTOMATIC, MANUAL
	}
	
	private Mode mode;
	
	/**
	 * Creates a BoxLifter object that has a middle switch.
	 * This will also set the lifter to be used in AUTOMATIC mode.
	 * @param motorIn Lift motor
	 * @param switchLowerIn Switch for lower point
	 * @param switchMiddleIn Switch for mid point
	 * @param switchHigherIn switch for high point
	 * @param playerIn MetalSkinsController to control BoxLifter
	 */
	public BoxLifter(SpeedController motorIn, DigitalInput switchLowerIn, DigitalInput switchMiddleIn, DigitalInput switchHigherIn, MetalSkinsController playerIn) {
		motorLifter = motorIn;
		motorLifter.setInverted(true);
		switchLower = switchLowerIn;
		switchMiddle = switchMiddleIn;
		switchHigher = switchHigherIn;
		player = playerIn;
		stateNow = State.BEGIN;
		mode = Mode.AUTOMATIC;
	}
	
	/**
	 * Creates a BoxLifter object that does not have a middle switch.
	 * This will also set the lifter to be used in MANUAL mode.
	 * @param motorIn Lift motor
	 * @param switchLowerIn Switch for lower point
	 * @param switchHigherIn switch for high point
	 * @param playerIn MetalSkinsController to control BoxLifter
	 */
	public BoxLifter(SpeedController motorIn, DigitalInput switchLowerIn, DigitalInput switchHigherIn, MetalSkinsController playerIn) {
		motorLifter = motorIn;
		motorLifter.setInverted(true);
		switchLower = switchLowerIn;
		switchHigher = switchHigherIn;
		player = playerIn;
		stateNow = State.BEGIN;
		mode = Mode.MANUAL;
	}
	
	/**
	 * This is the method that runs the box lifter in teleop.
	 * 
	 * <p>
	 * Call this method in teleop to run the box lifter using
	 * the State Machine mechanics and controller input.
	 * </p>
	 */
	public void run() {
		switch(stateNow) {
			case BEGIN:
				begin();
				break;
			case HIGH:
				high();
				break;
			case IDLE:
				idle();
				break;
			case LIFTING:
				lifting();
				break;
			case LOW:
				low();
				break;
			case LOWERING:
				lowering();
				break;
			case MIDDLE:
				middle();
				break;
			default:
				break;
			
		}
	}
	
	public boolean runTest() {
		switch(stateNow) {
			case BEGIN:
				beginTest();
				return false;
			case LIFTING:
				liftingTest();
				return false;
			case LOWERING:
				loweringTest();
				return false;
			case MIDDLE:
				return true;
			default:
				return false;
			
		}
	}
	
	/**
	 * Method for the BEGIN State of the box lifter state machine.
	 * It stops the lifter motor and then sets the state to LOW, 
	 * MIDDLE, HIGH or IDLE depending on whether any of the switches 
	 * are currently flipped
	 */
	private void begin() {
		stop();
		statePrevious = State.BEGIN;
			
		if(getSwitchLow()) {
			stateNow = State.LOW;
		} else if(getSwitchHigh()) {
			stateNow = State.HIGH;
		} else if(getSwitchMiddle()) {
			stateNow = State.MIDDLE;
		} else {
			stateNow = State.IDLE;
		}
	}
	
	public void initTest() {
		stateNow = State.BEGIN;
	}
	
	private void beginTest() {
		stop();
		statePrevious = State.BEGIN;
		
		if(getSwitchLow()) {
			rise();
			stateNow = State.LIFTING;
		} else {
			lower();
			stateNow = State.LOWERING;
		}
	}
	
	/**
	 * Method for the IDLE State of the box lifter state machine.
	 * This is the state where the lifter will not be moving.
	 * Based on controller inputs, the right bumper will lift the box
	 * Collector and set the state to LIFTING. The left bumper will 
	 * lower the box lifter and set the state to LOWERING.
	 */
	private void idle() {
		if(player.getBumper(Hand.kRight)) {
			rise();
			stateNow = State.LIFTING;
			statePrevious = State.IDLE;
		} else if(player.getBumper(Hand.kLeft)) {
			lower();
			stateNow = State.LOWERING;
			statePrevious = State.IDLE;
		}
	}
	
	/**
	 * Method for the LOWERING state of the box lifter state machine.
	 * This is the state where the lifter will be lowering.
	 * 
	 * <p>
	 * If the lifter is in automatic mode, it will lower until it hits the middle 
	 * switch where it will stop the motor and go to MIDDLE state, or until it 
	 * hits the low switch where it will stop the motor and go to LOW state.
	 * </p>
	 * <p>
	 * If the lifter is in manual mode, it will lower until either the low switch
	 * is hit and goes to LOW state or until the player lets go of the left bumper on
	 * the controller and goes to IDLE state.
	 * </p>
	 */
	private void lowering() {
		if(mode == Mode.AUTOMATIC) { // If mode is Automatic
			if(getSwitchMiddle() && statePrevious != State.MIDDLE) {
				stop();
				stateNow = State.MIDDLE;
				statePrevious = State.LOWERING;
			} else if(getSwitchLow()) {
				stop();
				stateNow = State.LOW;
				statePrevious = State.LOWERING;
			}
		} else { // If mode is Manual
			if(getSwitchLow()) { // If not at the bottom
				if(!player.getBumper(Hand.kLeft)) { //If player lets go of the down button
					stop(); //Turn motor off
					stateNow = State.IDLE; // Set State to Idle
					statePrevious = State.LOWERING;
				}
			} else { // Turn motor off and set State to Low
				stop();
				stateNow = State.LOW;
				statePrevious = State.LOWERING;
			}
		}
	}
	
	private void loweringTest() {
		if(statePrevious == State.HIGH) {
			if(getSwitchMiddle()) {
				stop();
				stateNow = State.MIDDLE;
				statePrevious = State.LOWERING;
			}
		} else if(getSwitchLow()) {
			rise();
			stateNow = State.LIFTING;
			statePrevious = State.LOW;
		}
	}
	
	/**
	 * Method for the LIFTING state of the box lifter state machine.
	 * This is the state where the lifter will be lifting.
	 * 
	 * <p>
	 * If the lifter is in automatic mode, it will lift until it hits the middle 
	 * switch where it will stop the motor and go to MIDDLE state, or until it 
	 * hits the high switch where it will stop the motor and go to HIGH state.
	 * </p>
	 * <p>
	 * If the lifter is in manual mode, it will lift until either the high switch
	 * is hit and goes to HIGH state or until the player lets go of the right bumper on
	 * the controller and goes to IDLE state.
	 * </p>
	 */
	private void lifting() {
		if(mode == Mode.AUTOMATIC) { // if mode is Automatic
			if(getSwitchMiddle() && statePrevious != State.MIDDLE) {
				stop();
				stateNow = State.MIDDLE;
				statePrevious = State.LIFTING;
			} else if(getSwitchHigh()) {
				stop();
				stateNow = State.HIGH;
				statePrevious = State.LIFTING;
			}
		} else { // if mode is Manual
			if(getSwitchHigh()) { //if not at the Top
				if(!player.getBumper(Hand.kRight)) { // If player lets go of the up button
					stop(); //Turn motor off
					stateNow = State.IDLE; // Set State to Idle
					statePrevious = State.LIFTING;
				}
			} else { // Turn off motor and set State to High
				stop();
				stateNow = State.HIGH;
				statePrevious = State.LIFTING;
			}
		}
	}
	
	private void liftingTest() {
		if(getSwitchHigh()) {
			lower();
			stateNow = State.LOWERING;
			statePrevious = State.HIGH;
		}
	}
	
	/**
	 * Method for the LOW state of the box lifter state machine.
	 * <p>
	 * This is the state where the lifter will be stopped in the
	 * low position. When the right bumper on the controller is pressed,
	 * The lifter will lift and go to the LIFTING state.
	 * </p>
	 */
	private void low() {
		if(player.getBumper(Hand.kRight)) {
			rise();
			stateNow = State.LIFTING;
			statePrevious = State.LOW;
		}
	}
	
	/**
	 * Method for the MIDDLE state of the box lifter state machine.
	 * <p>
	 * This is the state where the lifter will be stopped in the
	 * middle position. If the right bumper on the controller is pressed,
	 * The lifter will lift and go to the LIFTING state. If the left 
	 * bumper on the controller is pressed, the lifter will lower and
	 * go to the LOWERING state.
	 * </p>
	 */
	private void middle() {
		if(player.getBumper(Hand.kRight)) {
			rise();
			stateNow = State.LIFTING;
			statePrevious = State.MIDDLE;
		} else if(player.getBumper(Hand.kLeft)) {
			lower();
			stateNow = State.LOWERING;
			statePrevious = State.MIDDLE;
		}
	}
	
	/**
	 * Method for the HIGH state of the box lifter state machine.
	 * <p>
	 * This is the state where the lifter will be stopped in the
	 * high position. When the left bumper on the controller is pressed,
	 * The lifter will lower and go to the LOWERING state.
	 * </p>
	 */
	private void high() {
		if(player.getBumper(Hand.kLeft)) {
			lower();
			stateNow = State.LOWERING;
			statePrevious = State.HIGH;
		}
	}
	
	public State getState() {
		return stateNow;
	}
	
	/**
	 * Returns the current state of the Box Lifter State Machine as a String object
	 * @return Current state of the Box Lifter state machine as a String object
	 */
	public String getStateString() {
		switch(stateNow) {
			case BEGIN:
				return "BEGIN";
			case HIGH:
				return "HIGH";
			case IDLE:
				return "IDLE";
			case LIFTING:
				return "LIFTING";
			case LOW:
				return "LOW";
			case LOWERING:
				return "LOWERING";
			case MIDDLE:
				return "MIDDLE";
			default:
				return "NULL";
		}
	}
	
	/**
	 * Lowers the box lifter by setting the motor to -1 
	 */
	public void lower() {
		motorLifter.set(-1);
	}
	
	/**
	 * Lifts the box lifter by setting the motor to 1
	 */
	public void rise() {
		motorLifter.set(1);
	}
	
	/**
	 * Stops the box lifter by setting the motor to 0
	 */
	public void stop() {
		motorLifter.set(0);
	}
	
	/**
	 * Get the current status of the low position switch
	 * @return True if the switch is flipped, false otherwise.
	 */
	public boolean getSwitchLow() {
		return !switchLower.get(); //Normally Open
	}
	
	/**
	 * Get the current status of the high position switch
	 * @return True if the switch is flipped, false otherwise.
	 */
	public boolean getSwitchHigh() {
		return !switchHigher.get(); //Normally Open
	}
	
	/**
	 * Get the current status of the middle position switch
	 * @return True if the switch is flipped, false otherwise.
	 */
	public boolean getSwitchMiddle() {
		return !switchMiddle.get(); //Normally Open
	}
	
	public void reset() {
		stateNow = State.BEGIN;
	}
}
