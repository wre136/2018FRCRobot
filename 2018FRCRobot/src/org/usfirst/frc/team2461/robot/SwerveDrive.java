package org.usfirst.frc.team2461.robot;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.PIDController;

/**
 * <h1> Swerve Drive Class </h1>
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * <p>
 * 2018 season SwerveDrive drive train that is used to control the 4 SwerveMotors.
 * </p>
 */
public class SwerveDrive
{
	/**
	 * <ul>
	 * <li>motor[0] = Front Left</li>
	 *  <li> motor[1] = Front Right</li>
	 *  <li> motor[2] = Rear Left</li>
	 *   <li>motor[3] = Rear Right</li>
	 *   </ul>
	 */
	private SwerveMotor motor[] = new SwerveMotor[4];
	
	/**
	 * Power factor to scale the drive motors by.
	 * <p>Scale from 0.0 (no power at all) to 1.0 (full power!)</p>
	 */
	private double POWER_FACTOR = 0.75;
	
	/**
	 * List to hold all commands to be ran in driveAuto() method (Autonomous code)
	 */
	private LinkedList<SwerveDriveAutoCommand> autoCommands = new LinkedList<SwerveDriveAutoCommand>();
	
	/**
	 * Holds current command being used by driveAuto() method
	 */
	private SwerveDriveAutoCommand currentCommand;
	
	/**
	 * Creates Swerve Drive using 4 SwerveMotor objects
	 * @param motorFL Front-Left motor reference
	 * @param motorFR Front-Right motor reference
	 * @param motorRL Rear-Left motor reference
	 * @param motorRR Rear-Right motor reference
	 */
	public SwerveDrive(SwerveMotor motorFL, SwerveMotor motorFR, SwerveMotor motorRL, SwerveMotor motorRR)
	{
		motor = new SwerveMotor[]{motorFL, motorFR, motorRL, motorRR};
	}
	
	/**
	 * Creates Swerve Drive Object using 4-element SwerveMotor array
	 * @param motorIn 4-element SwerveMotor array <ol>
	 * <li>0 - Front Left SwerveMotor</li>
	 * <li>1 - Front Right SwerveMotor</li>
	 * <li>2 - Rear Left SwerveMotor</li>
	 * <li>3 - Rear Right SwerveMotor</li>
	 */
	public SwerveDrive(SwerveMotor[] motorIn)
	{
		motor = motorIn;
	}
	
	/**
	 * Used to drive the robot during TeleOp mode.
	 * <p>Initial code credit goes to <a href="https://github.com/strykeforce/thirdcoast">https://github.com/strykeforce/thirdcoast</a></p>
	 * @param player MetalSkinsController used to drive robot
	 */
	public void drive(MetalSkinsController player)
	{
		double forward = player.getStickLeft()[1];
		double strafe = -player.getStickLeft()[0]; // Changed it to negative to fix strafing issue
		double azimuth = player.getStickRight()[0];
		
		final double LENGTH = 1.0;
	    final double WIDTH = 1.0;
	    final double RADIUS = Math.hypot(LENGTH, WIDTH);

	    //Original Values
//	    final double a = strafe - azimuth * (LENGTH / RADIUS);
//	    final double b = strafe + azimuth * (LENGTH / RADIUS);
//	    final double c = forward - azimuth * (WIDTH / RADIUS);
//	    final double d = forward + azimuth * (WIDTH / RADIUS);
	    
	    //Corrected Values
	    final double a = strafe + azimuth * (LENGTH / RADIUS);
	    final double b = strafe - azimuth * (LENGTH / RADIUS);
	    final double c = forward + azimuth * (WIDTH / RADIUS);
	    final double d = forward - azimuth * (WIDTH / RADIUS);
	    
	    // wheel speed
	    double[] ws = new double[4];
	    ws[0] = Math.hypot(b, d);
	    ws[1] = Math.hypot(b, c);
	    ws[2] = Math.hypot(a, d);
	    ws[3] = Math.hypot(a, c);
	    
	    // wheel azimuth
	    double[] wa = new double[4];
	    wa[0] = Math.toDegrees(Math.atan2(b, d));
	    wa[1] = Math.toDegrees(Math.atan2(b, c));
	    wa[2] = Math.toDegrees(Math.atan2(a, d));
	    wa[3] = Math.toDegrees(Math.atan2(a, c));
	    
	    final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
	    if (maxWheelSpeed > 1.0) {
	      for (int i = 0; i < ws.length; i++) {
	        ws[i] /= maxWheelSpeed;
	      }
	    }
	    
	    for (int i = 0; i < motor.length; i++) {
	        motor[i].drive(wa[i], ws[i]*POWER_FACTOR);
	      }
	}
	
	/**
	 * Drives the drive train based off of the commands in the autoCommands Linked List.
	 * If there are no commands in the list, driveAuto will simply do nothing.
	 * Add commands to the Linked List by using the addAutoCommand() method
	 */
	public void driveAuto()
	{	
		if(currentCommand == null && !autoCommands.isEmpty()) //if there is no current command but there are still commands in the list
		{
			currentCommand = autoCommands.poll(); //grab the next command
			loadAutoCommand(currentCommand);
			
		} else if(autoCommands.isEmpty()) { //else if the list of commands is empty
			return; //if the list of commands is empty, stop running this method
		}
		
		//driveByAutoCommand();
		
		if(isDone()) //If the command is done (motors reached their setpoints)
		{
			currentCommand = null; //Get rid of current command //Get rid of current command
		} else {
			driveByAutoCommand(); // Sets the setpoints for the wheels and enables their PID loops
		}
	}
	
	/**
	 * Adds a SwerveDriveAutoCommand to the Linked List. This Linked List of commands
	 * will be used by the driveAuto() method to drive the drive train autonomously
	 * 
	 * @param command SwerveDriveAutoCommand command that tells the drive train how far to
	 * move or how much to turn. Create SwerveDriveAutoCommand commands using the
	 * SwerveDriveAutoCommandFactory class
	 */
	public void addAutoCommand(SwerveDriveAutoCommand command)
	{
		autoCommands.add(command);
	}
	
	/**
	 * Returns whether or not all the drive motors of the drive train have reached their specified distance
	 * @return Returns true if all the drive motors have reached their setpoints and reset (disabled) themselves
	 */
	public boolean isDone()
	{
		return motor[0].getDriveOntarget() && motor[1].getDriveOntarget() && motor[2].getDriveOntarget() && motor[3].getDriveOntarget();
		//return !motor[0].drivePID_IsEnable() && !motor[1].drivePID_IsEnable() && !motor[2].drivePID_IsEnable() && !motor[3].drivePID_IsEnable();
	}
	
	private void driveByAutoCommand()
	{	
		for(int i = 0; i < 4; i++)
		{
			motor[i].driveAuto();
		}
	}
	
	/**
	 * Gets the current turn angle for each wheel
	 * @return 4-element double array in the range of 0 to 359.99 <ol>
	 * <li>0 - Front Left Wheel Turn Angle</li>
	 * <li>1 - Front Right Wheel Turn Angle</li>
	 * <li>2 - Rear Left Wheel Turn Angle</li>
	 * <li>3 - Rear Right Wheel Turn Angle</li>
	 * </ol>
	 */
	public double[] getTurnEncoderAngles()
	{
		double[] angles = {0,0,0,0};
		for(int i = 0; i<4; i++)
		{
			angles[i] = motor[i].getDirection();
		}
		
		return angles;
	}
	
	/**
	 * Gets the current turn angle setpoint for each wheel
	 * @return 4-element double array in the range of 0 to 359.99 <ol>
	 * <li>0 - Front Left Wheel Turn Angle Setpoint</li>
	 * <li>1 - Front Right Wheel Turn Angle Setpoint</li>
	 * <li>2 - Rear Left Wheel Turn Angle Setpoint</li>
	 * <li>3 - Rear Right Wheel Turn Angle Setpoint</li>
	 * </ol>
	 */
	public double[] getTurnEncoderSetpoints()
	{
		double[] angles = {0,0,0,0};
		
		for(int i = 0; i < 4; i++)
		{
			angles[i] = motor[i].getDirectionSetPoint();
		}
		
		return angles;
	}
	
	/**
	 * Gets the difference from the turn angle the wheels
	 * are suppose to be at and what angle they currently
	 * are at
	 * @return 4-element double array <ol>
	 * <li>0 - Front Left Wheel Turn Error</li>
	 * <li>1 - Front Right Wheel Turn Error</li>
	 * <li>2 - Rear Left Wheel Turn Error</li>
	 * <li>3 - Rear Right Wheel Turn Error</li>
	 */
	public double[] getPIDTurnErrors()
	{
		double[] values = {0,0,0,0};
		for(int i = 0; i< 4; i++)
		{
			values[i] = motor[i].getPIDTurnError();
		}
		
		return values;
	}
	
	/**
	 * Gets the distance each wheel has traveled and returns it as a 4-element array
	 * @return 4-element double array <ol>
	 * <li> [0] Front Left Wheel Distance</li>
	 * <li> [1] Front Right Wheel Distance</li>
	 * <li> [2] Rear Left Wheel Distance</li>
	 * <li> [3] Rear Right Wheel Distance</li>
	 * </ol>
	 */
	public double[] getDistance()
	{
		double[] values = {0,0,0,0};
		for(int i = 0; i< 4; i++)
		{
			values[i] = motor[i].getDistance();
		}
		
		return values;
	}
	
	/**
	 * Gets the average distance traveled by all the wheels
	 * @return Double value that is the average of all the wheels
	 */
	public double getDistanceAvg() {
		double avg = 0;
		for(int i = 0; i< 4; i++)
		{
			avg += motor[i].getDistance();
		}
		
		return (avg/4);
	}
	
	/**
	 * Gets the distance setpoint for each wheel and returns it in a 4-elemnet array
	 * @return 4-element double array <ol>
	 * <li> [0] Front Left Wheel Distance Setpoint</li>
	 * <li> [1] Front Right Wheel Distance Setpoint</li>
	 * <li> [2] Rear Left Wheel Distance Setpoint</li>
	 * <li> [3] Rear Right Wheel Distance Setpoint</li>
	 * </ol>
	 */
	public double[] getDistanceSetpoints()
	{
		double[] values = {0,0,0,0};
		for(int i = 0; i< 4; i++)
		{
			values[i] = motor[i].getDistanceSetPoint();
		}
		
		return values;
	}
	
	/**
	 * Gets the current speed of each wheel in inches per second
	 * @return 4-element double array representing inches per second
	 * <ol>
	 * <li> [0] Front Left Wheel Speed</li>
	 * <li> [1] Front Right Wheel Speed</li>
	 * <li> [2] Rear Left Wheel Speed</li>
	 * <li> [3] Rear Right Wheel Speed</li>
	 * </ol>
	 */
	public double[] getDriveSpeed()
	{
		double[] values = {0,0,0,0};
		for(int i = 0; i< 4; i++)
		{
			values[i] = motor[i].getDriveSpeed();
		}
		
		return values;
	}
	
	public double[] getDriveSpeedError()
	{
		double[] values = {0,0,0,0};
		for(int i = 0; i< 4; i++)
		{
			values[i] = motor[i].getDriveSpeedError();
		}
		
		return values;
	}
	
	/**
	 * Resets distance traveled by the drive train to zero and disables the
	 * driving and turning PID loops
	 */
	public void reset() {
		for(int i = 0; i < 4; i++)
		{
			motor[i].resetPIDDrive();
			motor[i].resetPIDTurn();
		}
	}
	
	/**
	 * Gets the current loaded command that is driving the wheels
	 * @return 2-element double array <ol>
	 * <li>0 - Direction of just the first wheel</li>
	 * <li>1 - Distance the wheels are to travel</li>
	 * </ol>
	 */
	public double[] getCurrentCommand()
	{
		double value[] = {0, 0};
		
		if(currentCommand != null)
		{
			value[0] = currentCommand.getDirectionSetpoint()[0];
			value[1] = currentCommand.getDistanceSetpoint();
		}
		
		return value;
	}
	
	/**
	 * Method used to set the P, I, D, and F coefficients of the DRIVE PID loop
	 * @param motorNum Number of motor to set values to: <ul>
	 * <li>0 - Front Left Wheel</li>
	 * <li>1 - Front Right Wheel</li>
	 * <li>2 - Rear Left Wheel</li>
	 * <li>3 - Rear Right Wheel</li>
	 * @param P Proportial coefficient value
	 * @param I Integral coefficient value
	 * @param D Differential coefficient value
	 * @param F Feed Forward coefficient value
	 */
	public void setDrivePIDValues(int motorNum, double P, double I, double D, double F)
	{
		motor[motorNum].setDrivePIDValues(P, I, D, F);
	}
	
	/**
	 * Method used to set the P, I, and D coefficients of the TURN PID loop
	 * @param motorNum Number of motor to set values to: <ul>
	 * <li>0 - Front Left Wheel</li>
	 * <li>1 - Front Right Wheel</li>
	 * <li>2 - Rear Left Wheel</li>
	 * <li>3 - Rear Right Wheel</li>
	 * @param P Proportial coefficient value
	 * @param I Integral coefficient value
	 * @param D Differential coefficient value
	 */
	public void setTurnPIDValues(int motorNum, double P, double I, double D)
	{
		motor[motorNum].setTurnPIDValues(P, I, D);
	}
	
	public PIDController getTurnPID(int motorNumber)
	{
		return motor[motorNumber].getTurnPID();
	}
	
	public PIDController getDrivePID(int motorNumber)
	{
		return motor[motorNumber].getDrivePID();
	}
	
	/**
	 * Sets the distance each wheel is to travel
	 * @param distanceToTravel Distance in inches
	 */
	public void setDistanceSetpoint(double distanceToTravel) {
		for(int i = 0; i < 4; i++) {
			motor[i].setDistanceSetPoint(distanceToTravel);
		}
	}
	
	/**
	 * Sets the direction each wheel is to turn to
	 * @param direction Direction in degrees between 0 and 359.99
	 */
	public void setDirectionSetpoint(double direction) {
		for(int i = 0; i < 4; i++) {
			motor[i].setDirectionSetPoint(direction);
		}
	}
	
	/**
	 * Loads a swerveDriveAutoCommand and uses it to set the
	 * direction and distance of each wheel
	 * @param command SweverDriveAutoCommand
	 */
	private void loadAutoCommand(SwerveDriveAutoCommand command) {
		for(int i = 0; i < 4; i++)
		{
			motor[i].setDirectionSetPoint(command.getDirectionSetpoint()[i]);
			motor[i].setDistanceSetPoint(command.getDistanceSetpoint());
			motor[i].resetPIDDrive();
			motor[i].enableTurnPID();
		}
	}
	
	public LinkedList<SwerveDriveAutoCommand> getAutoCommandList() {
		return autoCommands;
	}
	
	/**
	 * Method to clear all SwerveDriveAutoCommands from the autoCommand list
	 * and removes the currently loaded autoCommand
	 */
	public void clearAutoCommands() {
		autoCommands.clear();
		currentCommand = null;
	}
}
