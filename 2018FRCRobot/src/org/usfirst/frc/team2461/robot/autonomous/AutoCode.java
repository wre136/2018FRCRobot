package org.usfirst.frc.team2461.robot.autonomous;

public interface AutoCode
{
	public void run();
	public String getStateString();
	public String getStatePreviousString();
	public void reset();
}
