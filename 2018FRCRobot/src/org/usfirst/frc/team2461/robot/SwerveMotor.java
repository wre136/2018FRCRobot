package org.usfirst.frc.team2461.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <h1> Swerve Motor Class </h1>
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * <p>
 * 2018 season Swerve Motor subsystem that is used in the Swerve Drive Subsystem for driving
 * </p>
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
	
	private double kPDrive = 0.001;
	private double kIDrive = 0;
	private double kDDrive = 0;
	private double kFDrive = 0;
	
	private double kPTurn = 0.01;
	private double kITurn = 0.001;
	private double kDTurn = 0.01;
	
	private double pidDrivePower = 0.65;
	
	private double testTime;
	private double timeNow;
	
	private enum WheelPosition {
		FLWheel, FRWheel, RLWheel, RRWheel
	}
	
	protected WheelPosition wheelPosition;
	
	private enum TestState {
		BEGIN, DRIVE_TEST, TURN_TEST, END
	}
	
	private TestState testState;
	
	//Constructors
	/**
	 * Creates a Swerve Motor object
	 * @param driveCanID CAN Bus ID for the TalonSRX that controls the drive motor
	 * @param turnPWMChannel PWM Channel that controls the turning motor
	 * @param encTurnIn MA3 type encoder that measures the turn angle
	 * @param encDriveIn Encoder to measure distance travel
	 */
	public SwerveMotor(int driveCanID, int turnPWMChannel, MA3Encoder encTurnIn, Encoder encDriveIn)
	{
		motorDrive = new WPI_TalonSRX(driveCanID);
		encDrive = encDriveIn;
		
		motorTurn = new Spark(turnPWMChannel);
		encTurn = encTurnIn;
		configureEncoders();
		setupPIDControllers();
	}
	
	/**
	 * Creates a Swerve Motor object
	 * @param driveCanID CAN Bus ID for the TalonSRX that controls the drive motor
	 * @param turnPWMChannel PWM Channel that controls the turning motor
	 * @param encTurnIn MA3 type encoder that measures the turn angle
	 * @param encDriveIn Encoder to measure distance travel
	 * @param inverted True will run the motor backwards when positive power is applied
	 */
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
	
	/**
	 * Creates a Swerve Motor object
	 * @param driveCanID The CAN Bus ID for the TalonSRX that controls the drive motor
	 * @param turnPWMChannel The PWM Channel that controls the turning motor
	 * @param encTurnAIChannel The Analog Channel for MA3 Encoder
	 * @param encDriveDIOChannel0 The <b>A</b> DigitalInput channel for distance encoder
	 * @param encDriveDIOChannel1 The <b>B</b> DigitalInput channel for distance encoder
	 */
	public SwerveMotor(int driveCanID, int turnPWMChannel, int encTurnAIChannel, int encDriveDIOChannel0, int encDriveDIOChannel1)
	{
		motorDrive = new WPI_TalonSRX(driveCanID);
		encDrive = new Encoder(encDriveDIOChannel0, encDriveDIOChannel1);
		
		motorTurn = new Spark (turnPWMChannel);
		encTurn = new MA3Encoder(encTurnAIChannel);
		configureEncoders();
		setupPIDControllers();		
	}
	
	/**
	 * Creates a Swerve Motor object
	 * @param driveCanID The CAN Bus ID for the TalonSRX that controls the drive motor
	 * @param turnPWMChannel The PWM Channel that controls the turning motor
	 * @param encTurnAIChannel The Analog Channel for MA3 Encoder
	 * @param encDriveDIOChannel0 The <b>A</b> DigitalInput channel for distance encoder
	 * @param encDriveDIOChannel1 The <b>B</b> DigitalInput channel for distance encoder
	 * @param inverted True will run the motor backwards when positive power is applied
	 */
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
	
	/**
	 * Creates a Swerve Motor object
	 * @param motorDriveIn The TalonSRX that controls the drive motor
	 * @param motorTurnIn The Spark that controls the turning motor
	 * @param encTurnIn MA3 type encoder that measures the turn angle
	 * @param encDriveIn Encoder to measure distance travel
	 */
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
	
	/**
	 * Creates a Swerve Motor object
	 * @param motorDriveIn The TalonSRX that controls the drive motor
	 * @param motorTurnIn The Spark that controls the turning motor
	 * @param encTurnIn MA3 type encoder that measures the turn angle
	 * @param encDriveIn Encoder to measure distance travel
	 * @param inverted True will run the motor backwards when positive power is applied
	 */
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
	/**
	 * Gets the current angle the turn encoder detects
	 * @return Angle the encoder detects in double format from 0 to 359.99 degrees
	 */
	public double getDirection()
	{
		return encTurn.getAngle();
	}
	
	/**
	 * Sets the direction setpoint that the turning PID loop will
	 * try to turn the wheel too
	 * @param setPoint A double value in the range of 0 to 359.99 degrees
	 */
	public void setDirectionSetPoint(double setPoint)
	{
		pidTurn.setSetpoint(setPoint);
	}
	
/**
 * Getting the direction the wheels are trying to get to
 * @return double value in the range of 0 to 359.99 degrees
 */
	public double getDirectionSetPoint()
	{
		return pidTurn.getSetpoint();
	}
	
	/**
	 * Getting how far the robot has moved
	 * @return double value represented as inches
	 */
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
	
	/**
	 * Getting how far the robot needs to move in inches
	 * @return double value represented as inches
	 */
	public double getDistanceSetPoint()
	{
		return pidDrive.getSetpoint();
	}
	
	/**
	 * Getting how far away the robot is from the set distance
	 * @return double value represented as inches
	 */
	public double getDistanceFromSetPoint()
	{
		return pidDrive.getError();
	}

	/**
	 * When positive power is applied to driver motor it will spin them one way or the other
	 * @param inverted true will spin the drive motor backwards, false will spin the drive motor forward
	 */
	private void setInverted(boolean inverted)
	{
		motorDrive.setInverted(inverted);
		encDrive.setReverseDirection(inverted);
		isInverted = inverted;
	}
	
	/**
	 * Shows whether the drive motors are inverted
	 * @return true shows the drive motor will go backwards, false will show the drive motor going forward
	 */
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
		pidDrive.setOutputRange(-pidDrivePower, pidDrivePower); //Set PID for Drive to output restricted power
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
	 * @param direction Direction in degree the wheel should turn. Value from 0 to 359.99
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

			if(pidDrive.onTarget()) { // if the wheels have driven as far as they need too
				if(encDrive.getStopped()) { // If the wheels have come to a stop
					return true; // Return that the driveAuto method is done
				}
			}
		}
		return false; // Return false if the drive wheels have not gone far enough and/or have not stopped
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
	
	/**
	 * Shows whether drivePID Loop is running or not
	 * @return Boolean True means the drivePID Loop is running, false means it is not running
	 */
	public boolean drivePID_IsEnable()
	{
		return pidDrive.isEnabled();
	}
	
	/**
	 * Shows whether turnPID Loop is running or not
	 * @return Boolean True means the turnPID Loop is running, false means it is not running
	 */
	public boolean turnPID_IsEnable()
	{
		return pidTurn.isEnabled();
	}
	
	/**
	 * Shows how far off a certain degree the turning motor is
	 * @return double value the difference between the direction where should be at and where it as in degrees
	 */
	public double getPIDTurnError()
	{
		return pidTurn.getError();
	}
	
	/**
	 * Getting how far the robot has moved
	 * @return double value represented as inches
	 */
	public double getDistance()
	{
		return encDrive.getDistance();
	}
	
	/**
	 * Getting the speed the drive motor is moving
	 * @return double value returns inches per second
	 */
	public double getDriveSpeed()
	{
		return encDrive.getRate();
	}
	
	/**
	 * Shows how far off a certain speed the drive motor is
	 * @return double value the difference between the speed where should be at and where it as in inches per second
	 */
	public double getDriveSpeedError()
	{
		return pidDrive.getError();
	}
	
	/**
	 * Shows what speed drive motor should be at
	 * @return double value showing the speed drive motor should be at
	 */
	public double getDriveSpeedSetpoint()
	{
		if(encDrive.getPIDSourceType() == PIDSourceType.kRate) {
			return pidDrive.getSetpoint();
		} else {
			return 0;
		}
		
	}
	
	/**
	 * resets PIDDrive and zeroes out are distance traveled
	 */
	public void resetPIDDrive()
	{
		pidDrive.reset();
		encDrive.reset();
	}
	
	/**
	 * Disables PIDTurn preventing the turn motors to turn 
	 */
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
	
	/**
	 * Method used to set P, I, D, and F components of the DrivePID Values
	 * @param P Double value between 0.0 and 1.0
	 * @param I Double value between 0.0 and 1.0
	 * @param D Double value between 0.0 and 1.0
	 * @param F Double value between 0.0 and 1.0
	 */
	public void setDrivePIDValues(double P, double I, double D, double F)
	{
		pidDrive.setPID(P, I, D, F);
	}
	
	/**
	 * Method used to set P, I, and D components of the TurnPID Values
	 * @param P Double value between 0.0 and 1.0
	 * @param I Double value between 0.0 and 1.0
	 * @param D Double value between 0.0 and 1.0
	 */
	public void setTurnPIDValues(double P, double I, double D)
	{
		pidTurn.setPID(P, I, D);
	}
	
	/**
	 * Returns the TurningPID Loop object
	 * @return PIDContoller Object representing the TurningPID Loop
	 */
	public PIDController getTurnPID()
	{
		return pidTurn;
	}
	
	/**
	 * Returns the DrivePID Loop object
	 * @return PIDContoller Object representing the DrivePID Loop
	 */
	public PIDController getDrivePID()
	{
		return pidDrive;
	}
	
	/**
	 * Stops the PIDDrive loop and the PIDTurn loop
	 */
	public void disable() {
		pidDrive.reset();
		pidTurn.reset();
	}
	
	/**
	 * Returns the law count of the Drive Encoder
	 * @return Int Value
	 */
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
	
	/**
	 * Method for the BEGIN State of the testing code.
	 * <p>
	 * Sets the future TestTime to the current time plus 3 second to it
	 * and then sets the state to DRIVE_TEST.
	 * </p>
	 */
	private void testBegin() {
		testState = TestState.DRIVE_TEST;
		testTime = Robot.timer.get() + 3;
	}
	
	/**
	 * Tests the motion of the drive motor by running it full forward and the down to zero within 3 seconds
	 * afterwards it disables Drive Motor and sets the future test time to the current time plus 3 seconds to it
	 * sets the state to TURN_TEST
	 */
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
	
	/**
	 * Tests the motion of the turning motor by Turning it to 120, 240, and 0 degrees within 3 seconds
	 * afterwards it disables Turning Motor and
	 * sets the state to END
	 */
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
	
	/**
	 * Returns whether the drive motor has gone the distance 
	 * that it needs to, has stopped and if it is enabled.
	 * @return True means the drive wheel is <b>enabled</b>, has gone the <b>distance it needs</b>
	 * and is currently <b>stopped<b/>
	 */
	public boolean getDriveOntarget() {
		return pidDrive.onTarget() && encDrive.getStopped() && pidDrive.isEnabled();
	}
	
	public void setWheelPositionEnum(WheelPosition wheelPositionIn) {
		wheelPosition = wheelPositionIn;
	}
	
	public String getWheelPosition() {
		return wheelPosition.name();
	}
}
