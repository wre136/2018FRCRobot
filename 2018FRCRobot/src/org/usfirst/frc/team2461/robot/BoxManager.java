package org.usfirst.frc.team2461.robot;


import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	private enum BoxLifterMode {
		AUTOMATIC, MANUAL
	}
	
	private BoxLifterMode boxLifterMode;
	
	private enum BoxCollectorState {
		BEGIN, REST, SUCK_IN, SPIT_OUT, SPIN_BOX, TOGGLE_ARMS
	}
	
	BoxCollectorState boxCollectorStateNow;
	BoxCollectorState boxCollectorStatePrevious;
	
	private enum BoxCollectorArmsState {
		BEGIN, REST, RETRACTED, DEPLOYED
	}
	
	BoxCollectorArmsState boxCollectorArmsState;
	private boolean armToggleLock = false;
	
	private enum BoxManagerTestState {
		BEGIN, EXTEND_ARM, LOWER_TO_LOW, RAISE_TO_HIGH, LOWER_TO_MID, ARM_MOTORS_SUCK_IN, ARM_MOTOTS_SPIT_OUT,
		ARM_MOTORS_STOP, REAR_MOTORS_SUCK_IN, REAR_MOTORS_SPIT_OUT, REAR_MOTORS_STOP, RETRACT, DONE
	}
	
	private BoxManagerTestState boxManagerTestState;
	private BoxManagerTestState boxManagerTestStatePrevious;
	
	public BoxManager(BoxCollector boxCollectorIn, BoxLifter boxLifterIn, MetalSkinsController playerIn) {
		boxCollector = boxCollectorIn;
		boxCollectorStateNow = BoxCollectorState.BEGIN;
		boxCollectorStatePrevious = BoxCollectorState.BEGIN;
		
		boxLifter = boxLifterIn;
		boxLifterStateNow = BoxLifterState.BEGIN;
		boxLifterStatePrevious = BoxLifterState.BEGIN;
		boxLifterMode = BoxLifterMode.AUTOMATIC;
		player = playerIn;
		
		boxManagerTestState = BoxManagerTestState.BEGIN;
		boxManagerTestStatePrevious = BoxManagerTestState.BEGIN;
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
			case SPIN_BOX:
				boxCollectorSpinBox();
				break;
			case TOGGLE_ARMS:
				boxCollectorToggleArms();
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
		
		if(boxLifterMode == BoxLifterMode.MANUAL && player.getAButton() && !armToggleLock) {
			armToggleLock = true;
			if(boxCollector.getArmsExtended()) {
				boxCollector.armsExtend();
			} else {
				boxCollector.armsRetract();
			}
		}
		
		armToggleLock = player.getAButton();
		
	}
	
	private void boxCollectorBegin()
	{
		boxCollector.armsRetract();
		boxCollector.setArmMotorsStop();
		boxCollector.setRearMotorsStop();
		boxCollectorStateNow = BoxCollectorState.REST;
		boxCollectorStatePrevious = BoxCollectorState.BEGIN;
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
			spinBox();
			boxCollectorStateNow = BoxCollectorState.SPIN_BOX;
			boxCollectorStatePrevious = BoxCollectorState.REST;
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
	
	private void boxCollectorSpinBox() {
		if(player.getTriggerAxis(Hand.kLeft) == 1 && player.getTriggerAxis(Hand.kRight) == 1) {
			return;
		} else {
			stopBoxSucker();
			boxCollectorStateNow = BoxCollectorState.REST;
			boxCollectorStatePrevious = BoxCollectorState.SPIN_BOX;
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
			boxCollector.setArmMotorsSuckIn();
		}
		
		boxCollector.setRearMotorsSuckIn();
	}
	
	public void spinBox() {
		if(boxLifter.getSwitchLow()) {
			boxCollector.setArmMotorsSpinBox();
		}
		
		boxCollector.setRearMotorsSpinBox();
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
	
	public void boxCollectorToggleArms() {
		if(!player.getAButton()) {
			boxCollectorStateNow = BoxCollectorState.REST;
			boxCollectorStatePrevious = BoxCollectorState.TOGGLE_ARMS;
		}
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
		if(boxLifterMode == BoxLifterMode.AUTOMATIC) { // If mode is Automatic
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
		if(boxLifterMode == BoxLifterMode.AUTOMATIC) { // if mode is Automatic
			if(boxLifter.getSwitchMiddle() && boxLifterStatePrevious != BoxLifterState.MIDDLE) {
				boxLifter.stop();
				boxCollector.armsRetract();
				boxLifterStateNow = BoxLifterState.MIDDLE;
				boxLifterStatePrevious = BoxLifterState.LIFTING;
			} else if(boxLifter.getSwitchHigh()) {
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.HIGH;
				boxLifterStatePrevious = BoxLifterState.LIFTING;
			}
		} else { // if mode is Manual
			if(boxLifter.getSwitchHigh()) { //if not at the Top
				boxLifter.stop();
				boxLifterStateNow = BoxLifterState.HIGH;
				boxLifterStatePrevious = BoxLifterState.LIFTING;
			} else { // Turn off motor and set State to High
				if(!player.getBumper(Hand.kRight)) { // If player lets go of the up button
					boxLifter.stop(); //Turn motor off
					boxLifterStateNow = BoxLifterState.IDLE; // Set State to Idle
					boxLifterStatePrevious = BoxLifterState.LIFTING;
				}
			}
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
		return boxCollectorStateNow.name();
	}
	
	/**
	 * Returns the previous state of the Box Collector State Machine as a String object
	 * @return Previous state of the Box Collector state machine as a String object
	 */
	public String getBoxCollectorStatePreviousString() {
		return boxCollectorStatePrevious.name();
	}
	
	/**
	 * Returns the current state of the Box Lifter State Machine as a String object
	 * @return Current state of the Box Lifter state machine as a String object
	 */
	public String getBoxLifterStateString() {
		return boxLifterStateNow.name();
	}
	
	/**
	 * Returns the previous state of the Box Lifter State Machine as a String object
	 * @return Previous state of the Box Lifter state machine as a String object
	 */
	public String getBoxLifterStatePreviousString() {
		return boxLifterStatePrevious.name();
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
	
	/**
	 * This method readys the Box Collector and Box Lifter subsystems for testing.
	 * This <b>MUST</b> be ran before runTest()
	 */
	public void initTest() {
		boxCollectorStateNow = BoxCollectorState.BEGIN;
		boxLifterStateNow = BoxLifterState.BEGIN;
		boxManagerTestState = BoxManagerTestState.BEGIN;
	}
	
	/**
	 * Performs tests of Box Collector and Box Lifter subsystems.
	 * <p>
	 * The Box Collector will test in the following order:
	 * <ol>
	 * <li>Extend the arms</li>
	 * <li>Lower Box Collector to low position if not already in low position</li>
	 * <li>Raise Box Collector to high position</li>
	 * <li>Lower Box Collector to middle position</li>
	 * <li>Run arm motors to suck in a box</li>
	 * <li>Run arm motors to spit out a box</li>
	 * <li>Stop arm motors</li>
	 * <li>Run rear motors to suck in a box</li>
	 * <li>Run rear motors to spit out a box</li>
	 * <li>Stop rear motors</li>
	 * <li>Retract arms</li>
	 * <ol>
	 * </p>
	 * @return Returns whether the tests are finished. True means they are done.
	 */
	public boolean runTest() {
		switch(boxManagerTestState) {
		case ARM_MOTORS_STOP:
			boxManagerTestArmMotorStop();
			return false;
		case ARM_MOTORS_SUCK_IN:
			boxManagerTestArmMotorsSuckIn();
			return false;
		case ARM_MOTOTS_SPIT_OUT:
			boxManagerTestArmMotorsSpitOut();
			return false;
		case BEGIN:
			boxManagerTestBegin();
			return false;
		case DONE:
			return true;
		case EXTEND_ARM:
			boxManagerTestExtend();
			return false;
		case LOWER_TO_LOW:
			boxManagerTestLowerToLow();
			return false;
		case LOWER_TO_MID:
			boxManagerTestLowerToMid();
			return false;
		case RAISE_TO_HIGH:
			boxManagerTestRaiseToHigh();
			return false;
		case REAR_MOTORS_SPIT_OUT:
			boxManagerTestRearMotorsSpitOut();
			return false;
		case REAR_MOTORS_STOP:
			boxManagerTestRearMotorsStop();
			return false;
		case REAR_MOTORS_SUCK_IN:
			boxManagerTestRearMotorsSuckIn();
			return false;
		case RETRACT:
			boxManagerTestRetract();
			return false;
		default:
			return false;	
		}
	}
	
	private void boxManagerTestBegin() {
		boxCollector.setArmMotorsStop();
		boxCollector.setRearMotorsStop();
		boxLifter.stop();
		boxCollector.armsExtend();
		boxManagerTestState = BoxManagerTestState.EXTEND_ARM;
		boxManagerTestStatePrevious = BoxManagerTestState.BEGIN;
		testTime = Robot.timer.get() + 1;
	}
	
	private void boxManagerTestExtend() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			if(!boxLifter.getSwitchLow()) {
				boxLifter.lower();
				boxManagerTestState = BoxManagerTestState.LOWER_TO_LOW;
				boxManagerTestStatePrevious = BoxManagerTestState.EXTEND_ARM;
			} else {
				boxLifter.rise();
				boxManagerTestState = BoxManagerTestState.RAISE_TO_HIGH;
				boxManagerTestStatePrevious = BoxManagerTestState.EXTEND_ARM;
			}
		}
	}
	
	private void boxManagerTestLowerToLow() {
		if(boxLifter.getSwitchLow()) {
			boxLifter.rise();
			boxManagerTestState = BoxManagerTestState.RAISE_TO_HIGH;
			boxManagerTestStatePrevious = BoxManagerTestState.LOWER_TO_LOW;
		}
	}
	
	private void boxManagerTestRaiseToHigh() {
		if(boxLifter.getSwitchHigh()) {
			boxLifter.lower();
			boxManagerTestState = BoxManagerTestState.LOWER_TO_MID;
			boxManagerTestStatePrevious = BoxManagerTestState.RAISE_TO_HIGH;
		}
	}
	
	private void boxManagerTestLowerToMid() {
		if(boxLifter.getSwitchLow()) {
			boxLifter.stop();
			boxCollector.setArmMotorsSuckIn();
			boxManagerTestState = BoxManagerTestState.ARM_MOTORS_SUCK_IN;
			boxManagerTestStatePrevious = BoxManagerTestState.LOWER_TO_MID;
			testTime = Robot.timer.get() + 2;
		}
	}
	
	private void boxManagerTestArmMotorsSuckIn() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			boxCollector.setArmMotorsSpitOut();
			boxManagerTestState = BoxManagerTestState.ARM_MOTOTS_SPIT_OUT;
			boxManagerTestStatePrevious = BoxManagerTestState.ARM_MOTORS_SUCK_IN;
			testTime = Robot.timer.get() + 2;
		}
	}
	
	private void boxManagerTestArmMotorsSpitOut() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			boxCollector.setArmMotorsStop();
			boxManagerTestState = BoxManagerTestState.ARM_MOTORS_STOP;
			boxManagerTestStatePrevious = BoxManagerTestState.ARM_MOTOTS_SPIT_OUT;
			testTime = Robot.timer.get() + 1;
		}
	}
	
	private void boxManagerTestArmMotorStop() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			boxCollector.setRearMotorsSuckIn();
			boxManagerTestState = BoxManagerTestState.REAR_MOTORS_SUCK_IN;
			boxManagerTestStatePrevious = BoxManagerTestState.ARM_MOTORS_STOP;
			testTime = Robot.timer.get() + 2;
		}
	}
	
	private void boxManagerTestRearMotorsSuckIn() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			boxCollector.setRearMotorsSpitOut();
			boxManagerTestState = BoxManagerTestState.REAR_MOTORS_SPIT_OUT;
			boxManagerTestStatePrevious = BoxManagerTestState.REAR_MOTORS_SUCK_IN;
			testTime = Robot.timer.get() + 2;
		}
	}
	
	private void boxManagerTestRearMotorsSpitOut() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			boxCollector.setArmMotorsStop();
			boxManagerTestState = BoxManagerTestState.REAR_MOTORS_STOP;
			boxManagerTestStatePrevious = BoxManagerTestState.REAR_MOTORS_SPIT_OUT;
			testTime = Robot.timer.get() + 1;
		}
	}
	
	private void boxManagerTestRearMotorsStop() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			boxCollector.armsRetract();
			boxManagerTestState = BoxManagerTestState.RETRACT;
			boxManagerTestStatePrevious = BoxManagerTestState.REAR_MOTORS_STOP;
			testTime = Robot.timer.get() + 1;
		}
	}
	
	private void boxManagerTestRetract() {
		timeNow = Robot.timer.get();
		if(timeNow > testTime) {
			boxManagerTestState = BoxManagerTestState.DONE;
			boxManagerTestStatePrevious = BoxManagerTestState.RETRACT;
		}
	}
	
	public void setBoxLifterModeAutomatic(boolean isAutomatic) {
		if(isAutomatic==true) {
			boxLifterMode = BoxLifterMode.AUTOMATIC;
		} else {
			boxLifterMode = BoxLifterMode.MANUAL;
		}
	}
	
	/**
	 * Writes information about a Box Manager to the SmartDashboard for review and
	 * troubleshooting
	 * @param item Item to debug. <ul><li>0 = Box Collector</li><li>1 = Box Lifter</li><li>2 = Box Manager Test State</li></ul>
	 */
	public void debug(int item) {
		if(item == 0) {
			SmartDashboard.putString("Box Collector State", getBoxCollectorStateString());
			SmartDashboard.putString("Box Collector Previous State", getBoxCollectorStatePreviousString());
		} else if(item == 1) {
			SmartDashboard.putString("Box Lifter State", getBoxLifterStateString());
			SmartDashboard.putString("Box Lifter Previous State", getBoxLifterStatePreviousString());
			SmartDashboard.putBoolean("Box Lifter High Switch", boxLifter.getSwitchHigh());
			SmartDashboard.putBoolean("Box Lifter Middle Switch", boxLifter.getSwitchMiddle());
			SmartDashboard.putBoolean("Box Lifter Low Switch", boxLifter.getSwitchLow());
		} else if(item == 2) {
			SmartDashboard.putString("Box Manager Test State", boxManagerTestState.name());
			SmartDashboard.putString("Box Manager Test Previous State", boxManagerTestStatePrevious.name());
		}
	}
}
