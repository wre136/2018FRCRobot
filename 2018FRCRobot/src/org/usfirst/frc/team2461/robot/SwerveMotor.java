package org.usfirst.frc.team2461.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;

/**
 * @author William R Edds FRC 2461 - The METAL-SKINs
 *
 */
public class SwerveMotor
{
	private MA3Encoder encTurn;
	private Encoder encDrive;
	private WPI_TalonSRX motorDrive;
	private Spark motorTurn;
	private boolean isInverted = false;
	private PIDController pidDrive;
	private PIDController pidTurn;
	private double setPointDrive = 0;
	
	private double kPDrive = 0.001;
	private double kIDrive = 0;
	private double kDDrive = 0;
	private double kFDrive = 0;
	
	private double kPTurn = 0.01;
	private double kITurn = 0.001;
	private double kDTurn = 0.01;
	
	private double testTime;
	private double timeNow;
	
	private enum TestState {
		BEGIN, DRIVE_TEST, TURN_TEST, END
	}
	
	private TestState testState;
	
	//Constructors
	public SwerveMotor(int driveCanID, int turnPWMChannel, MA3Encoder encTurnIn, Encoder encDriveIn)
	{
		motorDrive = new WPI_TalonSRX(driveCanID);
		encDrive = encDriveIn;
		
		motorTurn = new Spark(turnPWMChannel);
		encTurn = encTurnIn;
		configureEncoders();
		setupPIDControllers();
	}
	
	public SwerveMotor(int driveCanID, int turnPWMChannel, MA3Encoder encTurnIn, Encoder encDriveIn, boolean inverted)
	{
		motorDrive = new WPI_TalonSRX(driveCanID);
		encDrive = encDriveIn;
		setInverted(inverted);
		
		motorTurn = new Spark (turnPWMChannel);
		encTurn = encTurnIn;
		configureEncoders();
		setupPIDControllers();
	}
	
	public SwerveMotor(int driveCanID, int turnPWMChannel, int encTurnAIChannel, int encDriveDIOChannel0, int encDriveDIOChannel1)
	{
		motorDrive = new WPI_TalonSRX(driveCanID);
		encDrive = new Encoder(encDriveDIOChannel0, encDriveDIOChannel1);
		
		motorTurn = new Spark (turnPWMChannel);
		encTurn = new MA3Encoder(encTurnAIChannel);
		configureEncoders();
		setupPIDControllers();		
	}
	
	public SwerveMotor(int driveCanID, int turnPWMChannel, int encTurnAIChannel, int encDriveDIOChannel0, int encDriveDIOChannel1, boolean inverted)
	{
		motorDrive = new WPI_TalonSRX(driveCanID);
		encDrive = new Encoder(encDriveDIOChannel0, encDriveDIOChannel1);
		setInverted(inverted);
		
		motorTurn = new Spark (turnPWMChannel);
		encTurn = new MA3Encoder(encTurnAIChannel);
		configureEncoders();
		setupPIDControllers();
	}
	
	public SwerveMotor(WPI_TalonSRX motorDriveIn, Spark motorTurnIn, MA3Encoder encTurnIn, Encoder encDriveIn)
	{
		motorDrive = motorDriveIn;
		encDrive = encDriveIn;
		setInverted(motorDrive.getInverted());
		pidDrive = new PIDController(kPDrive, kIDrive, kDDrive, kFDrive, encDrive, motorDrive);
		
		motorTurn = motorTurnIn;
		encTurn = encTurnIn;
		configureEncoders();
		setupPIDControllers();
	}
	
	public SwerveMotor(WPI_TalonSRX motorDriveIn, Spark motorTurnIn, MA3Encoder encTurnIn, Encoder encDriveIn, boolean inverted)
	{
		motorDrive = motorDriveIn;
		encDrive = encDriveIn;
		setInverted(inverted);
		
		motorTurn = motorTurnIn;
		encTurn = encTurnIn;
		configureEncoders();
		setupPIDControllers();
	}
	
	//Accessors
	public double getDirection()
	{
		return encTurn.getAngle();
	}
	
	public void setDirectionSetPoint(double setPoint)
	{
		pidTurn.setSetpoint(setPoint);
	}
	
	public double getDirectionSetPoint()
	{
		return pidTurn.getSetpoint();
	}
	
	public double getDistanceTraveled()
	{
		return encDrive.getDistance();
	}
	
	/**
	 * Drive the motor a certain distance
	 * @param setPoint Setpoint in inches
	 */
	public void setDistanceSetPoint(double setPoint)
	{
		if(encDrive.getPIDSourceType() != PIDSourceType.kDisplacement)
		{
			encDrive.setPIDSourceType(PIDSourceType.kDisplacement);
		}
		
		pidDrive.setSetpoint(setPoint);
	}
	
	/**
	 * Run the wheels at a certain speed
	 * @param setPoint Setpoint in inches per second
	 */
	public void setDrivePowerSetpoint(double setPoint)
	{
		if(encDrive.getPIDSourceType() != PIDSourceType.kRate)
		{
			encDrive.setPIDSourceType(PIDSourceType.kRate);
		}
		
		pidDrive.setSetpoint(setPoint);
	}
	
	public double getDistanceSetPoint()
	{
		return setPointDrive;
	}
	
	public double getDistanceFromSetPoint()
	{
		return pidDrive.getError();
	}

	private void setInverted(boolean inverted)
	{
		motorDrive.setInverted(inverted);
		encDrive.setReverseDirection(inverted);
		isInverted = inverted;
	}
	
	public boolean getInverted()
	{
		return isInverted;
	}
	
	
	//Mutators
	private void configureEncoders() {
		encTurn.setPIDSourceType(PIDSourceType.kDisplacement);
		
		encDrive.setDistancePerPulse(0.094153);
		encDrive.setMaxPeriod(0.5);
		encDrive.setMinRate(0.2);
	}
	
	private void setupPIDControllers()
	{
		pidDrive = new PIDController(kPDrive, kIDrive, kDDrive, kFDrive, encDrive, motorDrive);
		pidDrive.setOutputRange(-0.25, 0.25); //Set PID for Drive to output 1/4th power
		pidDrive.setAbsoluteTolerance(5);
		
		pidTurn = new PIDController(kPTurn, kITurn, kDTurn, encTurn, motorTurn);
		pidTurn.setInputRange(0, 359);
		pidTurn.setOutputRange(-1, 1);
		pidTurn.setContinuous();
		pidTurn.setAbsoluteTolerance(3);
		pidTurn.enable();
		pidTurn.setEnabled(true);
	}
	
	/**
	 * Method to drive the wheel in normal teleop mode.
	 * @param direction Direction in degree the wheel should turn. Value from 0 to 359
	 * @param drivePower Speed wheel should be given. Value from -1 to 1
	 */
	public void drive(double direction, double drivePower)
	{		
		if(direction < 0) // If direction is negative
		{
			direction = -direction; // make direction that much from 360
		} else {
			direction = 360 - direction; // make direction that much from 360
		}
		
		setDirectionSetPoint(direction);
		
		setDrivePowerSetpoint(drivePower);
		
		
		
		if(!pidTurn.isEnabled())
		{
			pidTurn.enable();
		}
		
		if(pidTurn.onTarget())
		{
			motorDrive.set(drivePower);
			
		} else {
			motorDrive.set(drivePower);
		}
	}
	
	/**
	 * Method to call to drive robot solely by setpoints to the turn and drive PID Loops. This
	 * should be called by SwerveDrive and its autonomous code or other autonomous code. 
	 * @return Returns TRUE if the Drive PID loop is on target meaning it has traveled
	 * the distance that it was set too
	 */
	public boolean driveAuto()
	{
		if(!pidTurn.isEnabled()) // Make sure turn PID loop is enabled
		{
			pidTurn.enable();
		}
		
		if(pidTurn.onTarget()) // If the wheels are pointing in the correct direction
		{
			if(!pidDrive.isEnabled()) { // Make sure the drive PID loop is enabled
				pidDrive.enable();
			}
			
			if(pidDrive.onTarget()) { // if the wheels haven't driven as far as they need too
				if(encDrive.getStopped()) {
					pidDrive.reset();
					//encDrive.reset();
				}	
			} else { // If the wheels have gone the specified distance, reset and disable the PID loops
				//pidTurn.reset();
				
			}
		}
		
		return pidDrive.onTarget();
	}
	
	/**
	 * Will stop and disable the "turn" and "drive" PID loops. It will set the turn setpoints to where
	 * they are currently so they will not swing back to the zero degree spot
	 */
	public void stop()
	{
		pidTurn.setSetpoint(pidTurn.getSetpoint() - pidTurn.getError());
		pidTurn.disable();
		pidDrive.setSetpoint(pidDrive.getSetpoint() - pidDrive.getError());
		pidDrive.disable();
	}
	
	public boolean drivePID_IsEnable()
	{
		return pidDrive.isEnabled();
	}
	
	public boolean turnPID_IsEnable()
	{
		return pidTurn.isEnabled();
	}
	
	public double getPIDTurnError()
	{
		return pidTurn.getError();
	}
	
	public double getDistance()
	{
		return encDrive.getDistance();
	}
	
	public double getDriveSpeed()
	{
		return encDrive.getRate();
	}
	
	public double getDriveSpeedError()
	{
		return pidDrive.getError();
	}
	
	public double getDriveSpeedSetpoint()
	{
		if(encDrive.getPIDSourceType() == PIDSourceType.kRate) {
			return pidDrive.getSetpoint();
		} else {
			return 0;
		}
		
	}
	
	public void resetPIDDrive()
	{
		pidDrive.reset();
		encDrive.reset();
	}
	
	public void resetPIDTurn() {
		pidTurn.reset();
	}
	
	/**
	 * Enables the Drive PID Loop if not already enabled
	 */
	public void enableDrivePID()
	{
		if(!pidDrive.isEnabled())
		{
			pidDrive.enable();
		}
	}
	
	/**
	 * Enables the Turn PID Loop if not already enabled
	 */
	public void enableTurnPID()
	{
		if(!pidTurn.isEnabled())
		{
			pidTurn.enable();
		}
	}
	
	/**
	 * Enables both the Drive and Turn PID Loops if not already enabled
	 */
	/**
	 * 
	 */
	public void enableDriveAndTurnPID()
	{
		enableDrivePID();
		enableTurnPID();
	}
	
	public void setDrivePIDValues(double P, double I, double D, double F)
	{
		pidDrive.setPID(P, I, D, F);
	}
	
	public void setTurnPIDValues(double P, double I, double D)
	{
		pidTurn.setPID(P, I, D);
	}
	
	public PIDController getTurnPID()
	{
		return pidTurn;
	}
	
	public PIDController getDrivePID()
	{
		return pidDrive;
	}
	
	public void disable() {
		pidDrive.reset();
		pidTurn.reset();
	}
	
	public int getEncDriveCount() {
		return encDrive.get();
	}
	
	/**
	 * Performs a power test of the wheel followed by a turn test.
	 * The Drive test Goes from full forward power to off in 3 seconds.
	 * The Turn test Starts at 120 degrees, goes to 240 degrees and ends
	 * at 0, all within in 3 seconds
	 * @return Returns whether the test is finished. True means it is done.
	 */
	public boolean runTest() {
		switch(testState) {
			case BEGIN:
				testBegin();
				return false;
			case DRIVE_TEST:
				testDrive();
				return false;
			case END:
				return true;
			case TURN_TEST:
				testTurn();
				return false;
			default:
				return false;
			
		}
	}
	
	private void testBegin() {
		testState = TestState.DRIVE_TEST;
		testTime = Robot.timer.get() + 3;
	}
	
	private void testDrive() {
		timeNow = Robot.timer.get();
		if(timeNow < testTime) {
			double timeDiffernce = testTime - timeNow;
			this.drive(0, timeDiffernce/3);
		} else {
			this.drive(0, 0);
			this.disable();
			testState = TestState.TURN_TEST;
			testTime = Robot.timer.get() + 3;
		}
	}
	
	private void testTurn() {
		timeNow = Robot.timer.get();
		if(timeNow < testTime) {
			int timeDiffernce = (int)(testTime - timeNow);
			double angle;
			if(timeDiffernce < 1) {
				angle = 0;
			} else if (timeDiffernce < 2) {
				angle = 240;
			} else {
				angle = 120;
			}

			this.drive(angle, 0);
		} else {
			this.drive(0, 0);
			this.disable();
			testState = TestState.END;
		}
	}
	
	/**
	 * Run this to setup the wheel test. Run this once BEFORE EVER calling runTest()
	 */
	public void initTest() {
		testState = TestState.BEGIN;
	}
	
	public TestState getTestState() {
		return testState;
	}
}
