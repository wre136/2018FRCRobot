package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DigitalInput;
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
	
	public enum Mode {
		AUTOMATIC, MANUAL;
		
		public String toString() {
			if(this.ordinal() == 0) {
				return "AUTOMATIC";
			}
			else {
				return "MANUAL";
			}
		}
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
	public BoxLifter(SpeedController motorIn, DigitalInput switchLowerIn, DigitalInput switchMiddleIn, DigitalInput switchHigherIn) {
		motorLifter = motorIn;
		motorLifter.setInverted(true);
		switchLower = switchLowerIn;
		switchMiddle = switchMiddleIn;
		switchHigher = switchHigherIn;
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
	public BoxLifter(SpeedController motorIn, DigitalInput switchLowerIn, DigitalInput switchHigherIn) {
		motorLifter = motorIn;
		motorLifter.setInverted(true);
		switchLower = switchLowerIn;
		switchHigher = switchHigherIn;
		mode = Mode.MANUAL;
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
	
	public Mode getMode() {
		return mode;
	}
}
