package org.usfirst.frc.team2461.robot;

/**
 * @author William R Edds FRC 2461 - The METAL-SKINs
 *
 */
public class SwerveDriveAutoCommand
{
	private double distanceSetpoint;
	private double[] directionSetpoint;
	
	public SwerveDriveAutoCommand(double distance, double[] direction)
	{
		distanceSetpoint = distance;
		directionSetpoint = direction;
	}
	
	public double getDistanceSetpoint()
	{
		return distanceSetpoint;
	}
	
	public double[] getDirectionSetpoint()
	{
		return directionSetpoint;
	}
}
