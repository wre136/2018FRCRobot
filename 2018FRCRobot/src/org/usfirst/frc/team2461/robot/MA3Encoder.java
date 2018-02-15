package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * 
 * Class to read the MA3 Analog 10-bit Absolute encoders.
 *
 */
public class MA3Encoder extends SensorBase implements PIDSource
{
	private PIDSourceType m_pidSource = PIDSourceType.kDisplacement;
	private AnalogInput data;
	private int angleLast = 0;
	
	/**
	 * Creates MA3 Analog 10-bit Absolute Encoder
	 * @param analogChannel Analog channel on RoboRIO to use
	 */
	public MA3Encoder(int analogChannel)
	{
		data = new AnalogInput(analogChannel);
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource)
	{
		m_pidSource = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType()
	{
		return m_pidSource;
	}

	@Override
	public double pidGet()
	{
		return getAngle();
	}
	
	/**
	 * @return True: Angle is increasing   False: Angle is decreasing
	 */
	public boolean getDirection()
	{
		if(angleLast < getAngle())
		{
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * @return Returns analog value from analog input on RoboRIO in the range 0 to 4095
	 */
	public int getAnalogRead()
	{
		return data.getValue();
	}
	
	/**
	 * @return Returns decimal value of degrees the encoder measures
	 */
	public double getAngle()
	{
		int value = getAnalogRead(); //get AnalogVlaue from 0 to 4095 (Resolution of AO port on RoboRIO)
		value = map(value); //map AnalogValue to the range 0 to 1023 (resolution of actual encoder)
		return toAngle(value); //Convert encoder analog value to degrees and return it
	}
	
	private int map(int x)
	{
		int in_min = 6;
		int in_max = 4037;
		int out_min = 0;
		int out_max = 1023;
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
	
	/**
	 * @param analogValue
	 * @return
	 */
	private double toAngle(double analogValue)
	{
		double in_min = 0;
		double in_max = 1023;
		double out_min = 0;
		double out_max = 359;
		return (analogValue - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	@Override
	public void initSendable(SendableBuilder builder)
	{
		builder.setSmartDashboardType("Absolute Encoder");
		builder.addDoubleProperty("Angle", this::getAngle, null);
	}
}
