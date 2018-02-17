package org.usfirst.frc.team2461.robot;

/**
 * Class used to easily create SwerveDriveAutoCommands to be used for autonomous driving.
 * Can create commands for "Going Forward a distance", "Going Backward a distance",
 * "Turning Left a certain number of degrees" and "Turning Right a certain number of degrees"
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
	 * @param distance Distance in inches
	 * @return SwerveDriveAutoCommand Command to drive forward to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_GoForward(double distance)
	{
		double[] direction = {180,180,180,180};
		return new SwerveDriveAutoCommand(distance, direction) ;
	}
	
	/**
	 * @param distance Distance in inches
	 * @return SwerveDriveAutoCommand Command to drive backward to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_GoBackward(double distance)
	{
		double[] direction = {0,0,0,0};
		return new SwerveDriveAutoCommand(distance, direction) ;
	}
	
	/**
	 * @param degree Number of degree to turn too. 0.0 to 359.99
	 * @return SwerveDriveAutoCommand Command to "turn robot" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_TurnLeft(double degree)
	{
		double[] direction = {(360-135),(360-45),135,45};
		return new SwerveDriveAutoCommand((ROBOT_WIDTH*Math.toRadians(degree)), direction) ;
	}
	
	/**
	 * @param degree Number of degree to turn too. 0.0 to 359.99
	 * @return SwerveDriveAutoCommand Command to "turn robot" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_TurnRight(double degree)
	{
		double[] direction = {45,135,(360-45),(360-135)};
		return new SwerveDriveAutoCommand((ROBOT_WIDTH*Math.toRadians(degree)), direction) ;
	}
	
	/**
	 * @return SwerveDriveAutoCommand Command to "stop" to be put into
	 * SwerveDrive autoCommand Link List. Use this in conjunction with the 
	 * SwerveDrive addAutoCommand() method
	 */
	public static SwerveDriveAutoCommand command_Stop()
	{
		double[] direction = {0,0,0,0};
		return new SwerveDriveAutoCommand(0, direction) ;
	}
	
	public static SwerveDriveAutoCommand command_MoveLeft(double distance) {
		double[] direction = {90,90,90,90};
		return new SwerveDriveAutoCommand(distance, direction) ;
	}
	
	public static SwerveDriveAutoCommand command_MoveRight(double distance) {
		double[] direction = {270,270,270,270};
		return new SwerveDriveAutoCommand(distance, direction) ;
	}
}
