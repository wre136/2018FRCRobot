package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * <h1> Box Collector Class </h1>
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * <p>
 * 2018 season Box Collector subsystem that is used to suck in and spit out boxes
 * </p>
 */
public class BoxCollector
{
	private SpeedController motorIntakeLeft;
	private SpeedController motorIntakeRight;
	private SpeedController motorIntakeRear1;
	private SpeedController motorIntakeRear2;
	private DoubleSolenoid ramDeploy;
	private MetalSkinsController player;
	private BoxLifter boxLifter;
	private double testTime;
	private double timeNow;
	
	private enum State {
		BEGIN, DEPLOYING, RETRACT, REST, SUCK_IN, SPIT_OUT
	}
	
	private State stateNow;
	private State statePrevious;
	
	/**
	 * Creates a BoxCollector object.
	 * @param motorLeft Left motor on arm
	 * @param motorRight Right motor on arm
	 * @param motorRear Motor to run wheels inside the box
	 * @param ramIn DoubleSolenoid that open-closes the arms
	 * @param playerIn MetalSkinController to be used to activate the box collector
	 */
	public BoxCollector(SpeedController motorLeft, SpeedController motorRight, SpeedController motorRear1, SpeedController motorRear2, DoubleSolenoid ramIn, MetalSkinsController playerIn, BoxLifter boxLifterIn) {
		motorIntakeLeft = motorLeft;
		motorIntakeRight = motorRight;
		motorIntakeRear1 = motorRear1;
		motorIntakeRear2 = motorRear2;
		ramDeploy = ramIn;
		player = playerIn;
		boxLifter = boxLifterIn;
		stateNow = State.BEGIN;
	}
	
	/**
	 * This is the method that runs the box collector in teleop.
	 * 
	 * <p>
	 * Call this method in teleop to run the box collector using
	 * the State Machine mechanics and controller input.
	 * </p>
	 */
	public void run() {
		switch(stateNow) {
			case BEGIN:
				begin();
				break;
			case DEPLOYING:
				deploying();
				break;
			case REST:
				rest();
				break;
			case RETRACT:
				retract();
				break;
			case SPIT_OUT:
				spitOut();
				break;
			case SUCK_IN:
				suckIn();
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
			case RETRACT:
				return true;
			case SUCK_IN:
				suckInTest();
				return false;
			default:
				return false;
			
		}
	}
	
	public void initTest() {
		stateNow = State.BEGIN;
		armsExtend();
	}
	
	/**
	 * Method for the BEGIN State of the box collector state machine.
	 * <p>
	 * It stops the box collector motor, extends the arms, and sets
	 * the state to REST.
	 * </p>
	 */
	private void begin()
	{
		armsExtend();
		stopBoxSucker();
		stateNow = State.REST;
		statePrevious = State.BEGIN;
	}
	
	private void beginTest() {
		setMotorArms(-1);
		stateNow = State.SUCK_IN;
		statePrevious = State.BEGIN;
		testTime = Robot.timer.get() + 2;
	}
	
	/**
	 * Method for the DEPLOYING State of the box collector state machine.
	 * <p>
	 * It extends the arms so that the robot can grab boxes ahead of it.
	 * </p>
	 */
	private void deploying() {
		armsExtend();
		stateNow = State.REST;
		statePrevious = State.DEPLOYING;
	}
	
	/**
	 * Method for the REST State of the box collector state machine.
	 * <p>
	 * This is the state where the collector will not be doing anything.
	 * Based on controller inputs
	 * <ul>
	 * <li>The <b>right trigger</b> will suck boxes in and move
	 * the Collector state machine to SUCK_IN </li>
	 * <li>The <b>left trigger</b> will spit boxes out and move 
	 * the Collector state machine to SPIT_OUT </li> 
	 * </ul>
	 * </p>
	 */
	private void rest() {
		if(player.getTriggerAxis(Hand.kRight) == 1 && player.getTriggerAxis(Hand.kLeft) == 1) {
			return;
		} else if(player.getTriggerAxis(Hand.kRight) == 1) {
			suckBoxIn();
			stateNow = State.SUCK_IN;
		} else if(player.getTriggerAxis(Hand.kLeft) == 1) {
			spitBoxOut();
			stateNow = State.SPIT_OUT;
		} else if(player.getAButton()){
			armsRetract();
			stateNow = State.RETRACT;
		}
	}
	
	/**
	 * Method for the SUCK_IN State of the box collector state machine.
	 * <p>
	 * This is the state where the collector will be sucking in boxes.
	 * </p>
	 * <p>
	 * Based on controller inputs, letting go of the <b>right trigger</b> will stop the suction
	 *  of boxes and move the Collector state machine to REST
	 * </p>
	 */
	private void suckIn() {
		if(player.getTriggerAxis(Hand.kRight) == 1) {
			return;
		} else {
			stopBoxSucker();
			stateNow = State.REST;
			statePrevious = State.SUCK_IN;
		}
	}
	
	private void suckInTest() {
		timeNow = Robot.timer.get();
		
		if(timeNow >= (testTime + 6)) {
			setMotorRear(0);
			armsRetract();
			stateNow = State.RETRACT;
			statePrevious = State.SUCK_IN;
		} else if(timeNow >= (testTime + 4)) {
			setMotorRear(1);
		} else if(timeNow >= (testTime + 2)) {
			setMotorArms(0);
			setMotorRear(-1);
		} else if(timeNow >= testTime) {
			setMotorArms(1);
		}
	}
	
	/**
	 * Method for the SPIT_OUT State of the box collector state machine.
	 * <p>
	 * This is the state where the collector will be spitting out boxes.
	 * </p>
	 * <p>
	 * Based on controller inputs, letting go of the <b>left trigger</b> will 
	 * stop the secretion of boxes and move the Collector state machine to REST
	 * </p>
	 */
	private void spitOut() {
		if(player.getTriggerAxis(Hand.kLeft) == 1) {
			return;
		} else {
			setMotorSpeed(0);
			stateNow = State.REST;
			statePrevious = State.SPIT_OUT;
		}
	}
	
	/**
	 * Method for the RETRACT State of the box collector state machine.
	 * <p>
	 * It withdraws the arms so that they are protected and the robot 
	 * can not grab boxes ahead of it.
	 * </p>
	 * <p>
	 * Based on controller inputs, letting go of the <b>A Button</b> will 
	 * extend the arms and move the Collector state machine to REST
	 * </p>
	 */
	private void retract() {
		if(player.getAButton()) {
			return;
		} else {
			armsExtend();
			stateNow = State.REST;
			statePrevious = State.RETRACT;
		}
	}
	
	private void setMotorSpeed(double speed) {
		setMotorArms(speed);
		setMotorRear(speed);
	}
	
	private void setMotorArms(double speed) {
		if(boxLifter.getSwitchLow()) {
			motorIntakeLeft.set(speed);
			motorIntakeRight.set(-speed);
		} else {
			motorIntakeLeft.set(0);
			motorIntakeRight.set(0);
		}
	}
	
	private void setMotorRear(double speed) {
		motorIntakeRear1.set(speed);
		motorIntakeRear2.set(-speed);
	}
	
	public State getState() {
		return stateNow;
	}
	
	/**
	 * Returns the current state of the Box Collector State Machine as a String object
	 * @return Current state of the Box Collector state machine as a String object
	 */
	public String getStateString() {
		switch(stateNow) {
			case BEGIN:
				return "BEGIN";
			case DEPLOYING:
				return "DEPLOYING";
			case REST:
				return "REST";
			case RETRACT:
				return "RETRACT";
			case SPIT_OUT:
				return "SPIT_OUT";
			case SUCK_IN:
				return "SUCK_IN";
			default:
				return "NULL";
		}
	}
	
	public String getStatePreviousString() {
		switch(statePrevious) {
			case BEGIN:
				return "BEGIN";
			case DEPLOYING:
				return "DEPLOYING";
			case REST:
				return "REST";
			case RETRACT:
				return "RETRACT";
			case SPIT_OUT:
				return "SPIT_OUT";
			case SUCK_IN:
				return "SUCK_IN";
			default:
				return "NULL";
		}
	}
	
	/**
	 * Manual method to retract Box Collector Arms.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Box Collector State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void armsRetract() {
		ramDeploy.set(DoubleSolenoid.Value.kForward);
		stateNow = State.REST;
	}
	
	/**
	 * Manual method to extend Box Collector Arms.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Box Collector State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void armsExtend() {
		ramDeploy.set(DoubleSolenoid.Value.kReverse);
		stateNow = State.REST;
	}
	
	/**
	 * Manual method to spit out boxes.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Box Collector State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void spitBoxOut() {
		setMotorSpeed(-1);
	}
	
	/**
	 * Manual method to suck in boxes.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Box Collector State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void suckBoxIn() {
		setMotorSpeed(1);
	}
	
	/**
	 * Manual method to stop box suction and secretion.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Box Collector State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void stopBoxSucker() {
		setMotorSpeed(0);
	}
	
	public void reset() {
		stateNow = State.BEGIN;
	}
	
	public boolean getArmsExtended() {
		if(ramDeploy.get() == DoubleSolenoid.Value.kReverse) {
			return true;
		} else {
			return false;
		}
	}
}
