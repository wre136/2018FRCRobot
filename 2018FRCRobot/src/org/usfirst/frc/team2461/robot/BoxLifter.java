package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;

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
	
	private enum Mode {
		AUTOMATIC, MANUAL
	}
	
	private Mode mode;
	
	/**
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
	
	public BoxLifter(SpeedController motorIn, DigitalInput switchLowerIn, DigitalInput switchHigherIn, MetalSkinsController playerIn) {
		motorLifter = motorIn;
		motorLifter.setInverted(true);
		switchLower = switchLowerIn;
		switchHigher = switchHigherIn;
		player = playerIn;
		stateNow = State.BEGIN;
		mode = Mode.MANUAL;
	}
	
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
	
	private void begin() {
		motorLifter.set(0);
		stateNow = State.IDLE;
	}
	
	private void idle() {
		if(mode == Mode.AUTOMATIC) {
			if(!switchLower.get()) {
				stateNow = State.LOW;
			} else if(!switchMiddle.get()) {
				stateNow = State.MIDDLE;
			} else if(!switchHigher.get()) {
				stateNow = State.HIGH;
			} else if(player.getBumper(Hand.kRight)) {
				motorLifter.set(1);
				stateNow = State.LIFTING;
			} else if(player.getBumper(Hand.kLeft)) {
				motorLifter.set(-1);
				stateNow = State.LOWERING;
			}
		} else {
			if(!switchLower.get()) {
				stateNow = State.LOW;
			} else if(!switchHigher.get()) {
				stateNow = State.HIGH;
			} else if(player.getBumper(Hand.kRight)) {
				motorLifter.set(1);
				stateNow = State.LIFTING;
			} else if(player.getBumper(Hand.kLeft)) {
				motorLifter.set(-1);
				stateNow = State.LOWERING;
			}
		}
		
	}
	
	private void lowering() {
		if(mode == Mode.AUTOMATIC) { // If mode is Automatic
			if(!switchMiddle.get()) {
				motorLifter.set(0);
				stateNow = State.MIDDLE;
			} else if(!switchLower.get()) {
				motorLifter.set(0);
				stateNow = State.LOW;
			}
		} else { // If mode is Manual
			if(switchLower.get()) { // If not at the bottom
				if(!player.getBumper(Hand.kLeft)) { //If player lets go of the down button
					motorLifter.set(0); //Turn motor off
					stateNow = State.IDLE; // Set State to Idle
				}
			} else { // Turn motor off and set State to Low
				motorLifter.set(0);
				stateNow = State.LOW;
			}
		}
	}
	
	private void lifting() {
		if(mode == Mode.AUTOMATIC) { // if mode is Automatic
			if(!switchMiddle.get()) {
				motorLifter.set(0);
				stateNow = State.MIDDLE;
			} else if(switchHigher.get()) {
				motorLifter.set(0);
				stateNow = State.HIGH;
			}
		} else { // if mode is Manual
			if(switchHigher.get()) { //if not at the Top
				if(!player.getBumper(Hand.kRight)) { // If player lets go of the up button
					motorLifter.set(0); //Turn motor off
					stateNow = State.IDLE; // Set State to Idle
				}
			} else { // Turn off motor and set State to High
				motorLifter.set(0);
				stateNow = State.HIGH;
			}
		}
		
		
	}
	
	private void low() {
		if(player.getBumper(Hand.kRight)) {
			motorLifter.set(1);
			stateNow = State.LIFTING;
		}
	}
	
	private void middle() {
		if(player.getBumper(Hand.kRight)) {
			motorLifter.set(1);
			stateNow = State.LIFTING;
		} else if(player.getBumper(Hand.kLeft)) {
			motorLifter.set(-1);
			stateNow = State.LOWERING;
		}
	}
	
	private void high() {
		if(player.getBumper(Hand.kLeft)) {
			motorLifter.set(-1);
			stateNow = State.LOWERING;
		}
	}
	
	public State getState() {
		return stateNow;
	}
	
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
	
	public void lower() {
		motorLifter.set(-1);
	}
	
	public void rise() {
		motorLifter.set(1);
	}
	
	public void stop() {
		
	}
	
	public boolean getSwitchLow() {
		return switchLower.get();
	}
	
	public boolean getSwitchHigh() {
		return switchHigher.get();
	}
	
	public boolean getSwitchMiddle() {
		return switchMiddle.get();
	}
}
