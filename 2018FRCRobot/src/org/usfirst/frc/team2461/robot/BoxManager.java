package org.usfirst.frc.team2461.robot;


import edu.wpi.first.wpilibj.GenericHID.Hand;

public class BoxManager
{
	public BoxCollector boxCollector;
	public BoxLifter boxLifter;
	private MetalSkinsController player;
	private double testTime;
	private double timeNow;
	
	private enum BoxLifterState {
		BEGIN, IDLE, LOW, MIDDLE, HIGH, LIFTING, LOWERING
	}
	
	private BoxLifterState boxLifterStateNow;
	private BoxLifterState boxLifterStatePrevious;
	
	private enum BoxCollectorState {
		BEGIN, REST, SUCK_IN, SPIT_OUT
	}
	
	BoxCollectorState boxCollectorStateNow;
	BoxCollectorState boxCollectorStatePrevious;
	
	private enum BoxCollectorArmsState {
		RETRACTED, DEPLOYED
	}
	
	BoxCollectorArmsState boxCollectorArmsState;
	
	public BoxManager(BoxCollector boxCollectorIn, BoxLifter boxLifterIn, MetalSkinsController playerIn) {
		boxCollector = boxCollectorIn;
		boxCollectorStateNow = BoxCollectorState.BEGIN;
		
		boxLifter = boxLifterIn;
		boxLifterStateNow = BoxLifterState.BEGIN;
		player = playerIn;
	}
	
	/**
	 * This is the method that runs the Box Collector and Box Lifter in Teleop.
	 * 
	 * <p>
	 * Call this method in Teleop to run the above subsystems using
	 * the State Machine mechanics and controller input.
	 * </p>
	 */
	public void run()
	{
		switch(boxCollectorStateNow) {
			case BEGIN:
				boxCollectorBegin();
				break;
			case REST:
				boxCollectorRest();
				break;
			case SPIT_OUT:
				boxCollectorSpitOut();
				break;
			case SUCK_IN:
				boxCollectorSuckIn();
				break;
			default:
				break;
		}
		
		switch(boxLifterStateNow) {
			case BEGIN:
				boxLifterBegin();
				break;
			case HIGH:
				boxLifterHigh();
				break;
			case IDLE:
				boxLifterIdle();
				break;
			case LIFTING:
				boxLifterLifting();
				break;
			case LOW:
				boxLifterLow();
				break;
			case LOWERING:
				boxLifterLowering();
				break;
			case MIDDLE:
				boxLifterMiddle();
				break;
			default:
				break;
		}
	}
	
	/**
	 * Performs tests of Box Collector and Box Lifter subsystems.
	 * <p>
	 * The Box Collector will test in the following order:
	 * <ol>
	 * <li>Extend the arms</li>
	 * <li>Run arm motors to suck in a box</li>
	 * <li>Run arm motors to spit out a box</li>
	 * <li>Stop arm motors and run rear motors to suck in a box</li>
	 * <li>Run rear motors to spit out a box</li>
	 * <li>Stop rear motors and retract arms</li>
	 * 
	 * The Box Lifter will test in the following order:
	 * <ol>
	 * <li>Lower Box Collector to low position if not already in low position</li>
	 * <li>Raise Box Collector to high position</li>
	 * <li>Lower Box Collector to middle position</li>
	 * </p>
	 * @return Returns whether the tests are finished. True means they are done.
	 */
	public boolean runTest() {
		if(!runTestBoxCollector()) {
			return false;
		} else if(!runTestBoxLifter()) {
			return false;
		} else {
			return true;
		}
	}
	
	private boolean runTestBoxCollector() {
		switch(boxCollectorStateNow) {
			case BEGIN:
				boxCollectorBeginTest();
				return false;
			case REST:
				return true;
			case SUCK_IN:
				boxCollectorSuckInTest();
				return false;
			default:
				return false;
			
		}
	}
	
	private boolean runTestBoxLifter() {
		switch(boxLifterStateNow) {
			case BEGIN:
				boxLifterBeginTest();
				return false;
			case LIFTING:
				boxLifterLiftingTest();
				return false;
			case LOWERING:
				boxLifterLoweringTest();
				return false;
			case MIDDLE:
				return true;
			default:
				return false;
			
		}
	}
	
	private void boxCollectorBegin()
	{
		boxCollector.armsExtend();
		boxCollector.setArmMotorsStop();
		boxCollector.setRearMotorsStop();
		boxCollectorStateNow = BoxCollectorState.REST;
		boxCollectorStatePrevious = BoxCollectorState.BEGIN;
	}
	
	private void boxCollectorBeginTest() {
		boxCollector.setArmMotorsSuckIn();
		boxCollector.armsExtend();
		boxCollectorStateNow = BoxCollectorState.SUCK_IN;
		boxCollectorStatePrevious = BoxCollectorState.BEGIN;
		testTime = Robot.timer.get() + 2;
	}
	
	/**
	 * Method for the DEPLOYING State of the box collector arm state machine.
	 * <p>
	 * It extends the arms so that the robot can grab boxes ahead of it.
	 * </p>
	 */
	public void boxCollectorArmDeploy() {
		boxCollector.armsExtend();
		boxCollectorArmsState = BoxCollectorArmsState.DEPLOYED;
	}
	
	/**
	 * Method for the RETRACTING State of the box collector arm state machine.
	 * <p>
	 * It retracts the arms so that the arms are protected.
	 * </p>
	 */
	public void boxCollectorArmRetract() {
		boxCollector.armsRetract();
		boxCollectorArmsState = BoxCollectorArmsState.RETRACTED;
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
	private void boxCollectorRest() {
		if(player.getTriggerAxis(Hand.kRight) == 1 && player.getTriggerAxis(Hand.kLeft) == 1) {
			return;
		} else if(player.getTriggerAxis(Hand.kRight) == 1) {
			suckBoxIn();
			boxCollectorStateNow = BoxCollectorState.SUCK_IN;
			boxCollectorStatePrevious = BoxCollectorState.REST;
		} else if(player.getTriggerAxis(Hand.kLeft) == 1) {
			spitBoxOut();
			boxCollectorStateNow = BoxCollectorState.SPIT_OUT;
			boxCollectorStatePrevious = BoxCollectorState.REST;
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
	private void boxCollectorSuckIn() {
		if(player.getTriggerAxis(Hand.kRight) == 1) {
			return;
		} else {
			stopBoxSucker();
			boxCollectorStateNow = BoxCollectorState.REST;
			boxCollectorStatePrevious = BoxCollectorState.SUCK_IN;
		}
	}
	
	private void boxCollectorSuckInTest() {
		timeNow = Robot.timer.get();
		
		if(timeNow >= (testTime + 8)) {
			boxCollector.setRearMotorsStop();
			boxCollectorStateNow = BoxCollectorState.REST;
			boxCollectorStatePrevious = BoxCollectorState.SUCK_IN;
			
			boxCollectorArmRetract();
			
		} else if(timeNow >= (testTime + 6)) {
			boxCollector.setRearMotorsSpitOut();
		} else if(timeNow >= (testTime + 4)) {
			
			boxCollector.setRearMotorsSuckIn();
		} else if(timeNow >= (testTime + 2)) {
			boxCollector.setArmMotorsStop();
			
		} else if(timeNow >= testTime) {
			boxCollector.setArmMotorsSpitOut();
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
	private void boxCollectorSpitOut() {
		if(player.getTriggerAxis(Hand.kLeft) == 1) {
			return;
		} else {
			stopBoxSucker();
			boxCollectorStateNow = BoxCollectorState.REST;
			boxCollectorStatePrevious = BoxCollectorState.SPIT_OUT;
		}
	}
	
	/**
	 * Method to spit a box out of the Box Collector.
	 * <p>
	 * If the low switch of the lifter is flipped, both 
	 * the arm motors and rear motors will spit the box
	 * out. Otherwise only the rear motors will spit 
	 * the box out and the arm motors will remain off.
	 * </p>
	 */
	public void spitBoxOut() {
		if(boxLifter.getSwitchLow()) {
			boxCollector.setArmMotorsSpitOut();
		}
		
		boxCollector.setRearMotorsSpitOut();
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
		if(boxLifter.getSwitchLow()) {
			boxCollector.setArmMotorsSuckIn();;
		}
		
		boxCollector.setRearMotorsSuckIn();
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
		boxCollector.setArmMotorsStop();
		boxCollector.setRearMotorsStop();
	}
	
	private void boxLifterBegin() {
		boxLifter.stop();
		boxLifterStatePrevious = BoxLifterState.BEGIN;
			
		if(boxLifter.getSwitchLow()) {
			boxCollectorArmDeploy();
			boxLifterStateNow = BoxLifterState.LOW;
		} else if(boxLifter.getSwitchHigh()) {
			boxCollectorArmRetract();
			boxLifterStateNow = BoxLifterState.HIGH;
		} else if(boxLifter.getSwitchMiddle()) {
			boxCollectorArmRetract();
			boxLifterStateNow = BoxLifterState.MIDDLE;
		} else {
			boxLifterStateNow = BoxLifterState.IDLE;
		}
	}
	
	/**
	 * This method readys the Box Collector and Box Lifter subsystems for testing.
	 * This <b>MUST</b> be ran before runTest()
	 */
	public void initTest() {
		boxCollectorStateNow = BoxCollectorState.BEGIN;
		boxLifterStateNow = BoxLifterState.BEGIN;
	}
	
	private void boxLifterBeginTest() {
		boxLifter.stop();
		boxCollectorArmDeploy();
		boxLifterStatePrevious = BoxLifterState.BEGIN;
		
		if(boxLifter.getSwitchLow()) {
			boxLifter.rise();
			boxLifterStateNow = BoxLifterState.LIFTING;
		} else {
			boxLifter.lower();
			boxLifterStateNow = BoxLifterState.LOWERING;
		}
	}
	
	/**
	 * Method for the IDLE State of the box lifter state machine.
	 * This is the state where the lifter will not be moving.
	 * Based on controller inputs, the right bumper will lift the box
	 * Collector and set the state to LIFTING. The left bumper will 
	 * lower the box lifter and set the state to LOWERING.
	 */
	private void boxLifterIdle() {
		if(player.getBumper(Hand.kRight)) {
			boxLifter.rise();
			boxLifterStateNow = BoxLifterState.LIFTING;
			boxLifterStatePrevious = BoxLifterState.IDLE;
		} else if(player.getBumper(Hand.kLeft)) {
			boxLifter.lower();
			boxLifterStateNow = BoxLifterState.LOWERING;
			boxLifterStatePrevious = BoxLifterState.IDLE;
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
	private void boxLifterLowering() {
		if(boxLifter.getMode() == BoxLifter.Mode.AUTOMATIC) { // If mode is Automatic
			if(boxLifter.getSwitchMiddle() && boxLifterStatePrevious != BoxLifterState.MIDDLE) {
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.MIDDLE;
				boxLifterStatePrevious = BoxLifterState.LOWERING;
			} else if(boxLifter.getSwitchLow()) {
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.LOW;
				boxLifterStatePrevious = BoxLifterState.LOWERING;
			}
		} else { // If mode is Manual
			if(boxLifter.getSwitchLow()) { // If not at the bottom
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.LOW;
				boxLifterStatePrevious = BoxLifterState.LOWERING;
			} else { // Turn motor off and set State to Low
				if(!player.getBumper(Hand.kLeft)) { //If player lets go of the down button
					boxLifter.stop(); //Turn motor off
					boxLifterStateNow = BoxLifterState.IDLE; // Set State to Idle
					boxLifterStatePrevious = BoxLifterState.LOWERING;
				}
			}
		}
	}
	
	private void boxLifterLoweringTest() {
		if(boxLifterStatePrevious == BoxLifterState.HIGH) {
			if(boxLifter.getSwitchMiddle()) {
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.MIDDLE;
				boxLifterStatePrevious = BoxLifterState.LOWERING;
			}
		} else if(boxLifter.getSwitchLow()) {
			boxLifter.rise();
			boxLifterStateNow = BoxLifterState.LIFTING;
			boxLifterStatePrevious = BoxLifterState.LOW;
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
	private void boxLifterLifting() {
		if(boxLifter.getMode() == BoxLifter.Mode.AUTOMATIC) { // if mode is Automatic
			if(boxLifter.getSwitchMiddle() && boxLifterStatePrevious != BoxLifterState.MIDDLE) {
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.MIDDLE;
				boxLifterStatePrevious = BoxLifterState.LIFTING;
			} else if(boxLifter.getSwitchHigh()) {
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.HIGH;
				boxLifterStatePrevious = BoxLifterState.LIFTING;
			}
		} else { // if mode is Manual
			if(boxLifter.getSwitchHigh()) { //if not at the Top
				if(!player.getBumper(Hand.kRight)) { // If player lets go of the up button
					boxLifter.stop(); //Turn motor off
					boxLifterStateNow = BoxLifterState.IDLE; // Set State to Idle
					boxLifterStatePrevious = BoxLifterState.LIFTING;
				}
			} else { // Turn off motor and set State to High
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.HIGH;
				boxLifterStatePrevious = BoxLifterState.LIFTING;
			}
		}
	}
	
	private void boxLifterLiftingTest() {
		if(boxLifter.getSwitchHigh()) {
			boxLifter.lower();
			boxLifterStateNow = BoxLifterState.LOWERING;
			boxLifterStatePrevious = BoxLifterState.HIGH;
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
	private void boxLifterLow() {
		if(player.getBumper(Hand.kRight)) {
			boxLifter.rise();
			boxLifterStateNow = BoxLifterState.LIFTING;
			boxLifterStatePrevious = BoxLifterState.LOW;
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
	private void boxLifterMiddle() {
		if(player.getBumper(Hand.kRight)) {
			boxLifter.rise();
			boxCollectorArmRetract();
			boxLifterStateNow = BoxLifterState.LIFTING;
			boxLifterStatePrevious = BoxLifterState.MIDDLE;
		} else if(player.getBumper(Hand.kLeft)) {
			boxLifter.lower();
			boxCollectorArmDeploy();
			boxLifterStateNow = BoxLifterState.LOWERING;
			boxLifterStatePrevious = BoxLifterState.MIDDLE;
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
	private void boxLifterHigh() {
		if(player.getBumper(Hand.kLeft)) {
			boxLifter.lower();
			boxLifterStateNow = BoxLifterState.LOWERING;
			boxLifterStatePrevious = BoxLifterState.HIGH;
		}
	}
	
	/**
	 * Returns the current state of the Box Collector State Machine as a String object
	 * @return Current state of the Box Collector state machine as a String object
	 */
	public String getBoxCollectorStateString() {
		switch(boxCollectorStateNow) {
			case BEGIN:
				return "BEGIN";
			case REST:
				return "REST";
			case SPIT_OUT:
				return "SPIT_OUT";
			case SUCK_IN:
				return "SUCK_IN";
			default:
				return "NULL";
		}
	}
	
	/**
	 * Returns the previous state of the Box Collector State Machine as a String object
	 * @return Previous state of the Box Collector state machine as a String object
	 */
	public String getBoxCollectorStatePreviousString() {
		switch(boxCollectorStatePrevious) {
			case BEGIN:
				return "BEGIN";
			case REST:
				return "REST";
			case SPIT_OUT:
				return "SPIT_OUT";
			case SUCK_IN:
				return "SUCK_IN";
			default:
				return "NULL";
		}
	}
	
	/**
	 * Returns the current state of the Box Lifter State Machine as a String object
	 * @return Current state of the Box Lifter state machine as a String object
	 */
	public String getBoxLifterStateString() {
		switch(boxLifterStateNow) {
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
	 * Returns the previous state of the Box Lifter State Machine as a String object
	 * @return Previous state of the Box Lifter state machine as a String object
	 */
	public String getBoxLifterStatePreviousString() {
		switch(boxLifterStatePrevious) {
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
	 * Resets the Box Collector and Box Lifter State Machines to their BEGIN
	 * states.
	 */
	public void reset() {
		boxCollectorStatePrevious = boxCollectorStateNow;
		boxCollectorStateNow = BoxCollectorState.BEGIN;
		boxLifterStatePrevious = boxLifterStateNow;
		boxLifterStateNow = BoxLifterState.BEGIN;
	}
}
