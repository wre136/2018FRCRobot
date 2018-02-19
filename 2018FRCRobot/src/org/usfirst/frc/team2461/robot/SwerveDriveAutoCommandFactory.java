package org.usfirst.frc.team2461.robot;

/**
 * Class used to easily create SwerveDriveAutoCommands to be used for autonomous driving.
 * Can create commands for "Going Forward a distance", "Going Backward a distance",
 * "Turning Left a certain number of degrees", "Turning Right a certain number of degrees",
 * "Strafing Left a distance", and "Strafing Right a distance"
 * @author William R Edds FRC 2461 - The METAL-SKINs
 *
 */
public class SwerveDriveAutoCommandFactory
{
	private static SwerveDriveAutoCommandFactory swerve = new SwerveDriveAutoCommandFactory();
	private static final double ROBOT_WIDTH = 32.5;
	
	private SwerveDriveAutoCommandFactory()
	{
		
	}
	
	/**
	 * Method used to return the Singleton instance of the class
	 * @return Singleton instance of the class
	 */
	public static SwerveDriveAutoCommandFactory getInstance()
	{
		return swerve;
	}
	
	/**
	 * Method used to generate a SwerveDriveAutoCommand to go forward the
	 * specified distance
	 * @param distance Distance in inches
	 * @return SwerveDriveAutoCommand Command to drive forward to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_GoForward(double distance)
	{
		double[] direction = {180,180,180,180};
		return new SwerveDriveAutoCommand("Go Forward", distance, direction) ;
	}
	
	/**
	 * Method used to generate a SwerveDriveAutoCommand to go backward the
	 * specified distance
	 * @param distance Distance in inches
	 * @return SwerveDriveAutoCommand Command to drive backward to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_GoBackward(double distance)
	{
		double[] direction = {0,0,0,0};
		return new SwerveDriveAutoCommand("Go Backward", distance, direction) ;
	}
	
	/**
	 * Method used to generate a SwerveDriveAutoCommand to turn left the
	 * specified number of degrees
	 * @param degree Number of degree to turn too. 0.0 to 359.99
	 * @return SwerveDriveAutoCommand Command to "turn robot" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_TurnLeft(double degree)
	{
		double[] direction = {(360-135),(360-45),135,45};
		return new SwerveDriveAutoCommand("Turn Left", (ROBOT_WIDTH*Math.toRadians(degree)), direction) ;
	}
	
	/**
	 * Method used to generate a SwerveDriveAutoCommand to turn right the
	 * specified number of degrees
	 * @param degree Number of degree to turn too. 0.0 to 359.99
	 * @return SwerveDriveAutoCommand Command to "turn robot" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_TurnRight(double degree)
	{
		double[] direction = {45,135,(360-45),(360-135)};
		return new SwerveDriveAutoCommand("Turn Right", (ROBOT_WIDTH*Math.toRadians(degree)), direction) ;
	}
	
	/**
	 * Method used to generate a SwerveDriveAutoCommand to stop all movement
	 * @return SwerveDriveAutoCommand Command to "stop" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_Stop()
	{
		double[] direction = {0,0,0,0};
		return new SwerveDriveAutoCommand("Stop", 0, direction) ;
	}
	
	/**
	 * Method used to generate a SwerveDriveAutoCommand to strife left the
	 * specified distance
	 * @param distance Distance in inches
	 * @return SwerveDriveAutoCommand Command to "strife left" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_MoveLeft(double distance) {
		double[] direction = {90,90,90,90};
		return new SwerveDriveAutoCommand("Strafe Left", distance, direction) ;
	}
	
	/**
	 * Method used to generate a SwerveDriveAutoCommand to strife right the
	 * specified distance
	 * @param distance Distance in inches
	 * @return SwerveDriveAutoCommand Command to "strife right" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_MoveRight(double distance) {
		double[] direction = {270,270,270,270};
		return new SwerveDriveAutoCommand("Strafe Right", distance, direction) ;
	}
}
