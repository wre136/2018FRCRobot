package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * @author William R Edds FRC 2461 - The METAL-SKINs
 *
 */
public class MetalSkinsController extends XboxController
{
	private double precision = 0.15;
	private boolean ramp = false;
	
	/**
	 * @param port USB device on driver station
	 * @param ramped Whether analog stick outputs should be ramped parabolically
	 */
	public MetalSkinsController(final int port, boolean ramped)
	{
		super(port);
		ramp = ramped;
	}
	
	@Override
	public double getX(Hand hand)
	{
		double value = super.getX(hand);
		
		if(value < precision && value > -precision)
		{
			value = 0;
		}
		
		if(ramp)
		{
			if(value < 0)
			{
				value = -(Math.pow(value, 2));
			} else {
				value = Math.pow(value, 2);
			}
		}
		
		return value;
	}
	
	@Override
	public double getY(Hand hand)
	{
		double value = super.getY(hand);
		
		if(value < precision && value > -precision)
		{
			value = 0;
		}
		
		if(ramp)
		{
			if(value < 0)
			{
				value = -(Math.pow(value, 2));
			} else {
				value = Math.pow(value, 2);
			}
		}
		
		return value;
	}
	
	/**
	 * Gets both the X and Y values from the left stick of a MetalSkinsController object
	 * @return 2-Element Array holding X value and Y value of Left Stick on MetalSkins Controller
	 */
	public double[] getStickLeft()
	{
		double[] values = {getX(Hand.kLeft), getY(Hand.kLeft)};
		return values;
	}
	
	/**
	 * Gets both the X and Y values from the right stick of a MetalSkinsController object
	 * @return 2-Element Array holding X value and Y value of Left Stick on MetalSkins Controller
	 */
	public double[] getStickRight()
	{
		double[] values = {getX(Hand.kRight), getY(Hand.kRight)};
		return values;
	}
	
	/**
	 * Method that returns whether the X and Y values of the joy sticks are being
	 * ramped parabolically so that it is less sensitive as the sticks are in their natural
	 * positions
	 * @return Returns TRUE if ramp is being applied
	 */
	public boolean getRamp()
	{
		return ramp;
	}
	
	public double getPercision()
	{
		return precision;
	}
	
	/**
	 * Used to set the deadband of X and Y axes for the joy sticks. Value is range of
	 * 0 to 1 (zero being no deadband and 1 being evenything is deadband and does nothing).
	 * Suggest starting at value of 0.15
	 * @param value Value to det Deadband to. Range from 0.00 to 1.00
	 */
	public void setPercision(double value)
	{
		if(value >= 0)
		{
			precision = value;
		}
	}
}
