package org.usfirst.frc.team2461.robot;

/**
 * @author William R Edds FRC 2461 - The METAL-SKINs
 *
 */
public class SwerveDriveAutoCommand
{
	private double distanceSetpoint;
	private double[] directionSetpoint;
	private String commandName;
	
	public SwerveDriveAutoCommand(String commandNameIn, double distance, double[] direction)
	{
		commandName = commandNameIn;
		distanceSetpoint = distance;
		directionSetpoint = direction;
	}
	
	public String getCommandName()
	{
		return commandName;
	}

	public double getDistanceSetpoint()
	{
		return distanceSetpoint;
	}
	
	public double[] getDirectionSetpoint()
	{
		return directionSetpoint;
	}
	
	public String SwerveDriveAutoCommandToString() {
		return distanceSetpoint + "";
	}
}
