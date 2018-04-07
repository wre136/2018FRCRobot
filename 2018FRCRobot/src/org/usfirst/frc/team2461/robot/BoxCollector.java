package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * <h1> Box Collector Class </h1>
 * @author William R Edds FRC 2461 - The METAL-SKINs
 * <p>
 * 2018 season Box Collector subsystem that is used to suck in and spit out boxes
 * </p>
 */
public class BoxCollector
{
	private SpeedController motorIntakeArmLeft;
	private SpeedController motorIntakeArmRight;
	private SpeedController motorIntakeRearLeft;
	private SpeedController motorIntakeRearRight;
	private Solenoid ramDeploy1;
	private Solenoid ramDeploy2;
	
	/**
	 * Creates a BoxCollector object.
	 * @param motorLeft Left motor on arm
	 * @param motorRight Right motor on arm
	 * @param motorRear Motor to run wheels inside the box
	 * @param ramIn DoubleSolenoid that open-closes the arms
	 * @param playerIn MetalSkinController to be used to activate the box collector
	 */
	public BoxCollector(SpeedController motorArmLeft, SpeedController motorArmRight, SpeedController motorRearLeft, SpeedController motorRearRight, Solenoid ram1In, Solenoid ram2In) {
		motorIntakeArmLeft = motorArmLeft;
		motorIntakeArmRight = motorArmRight;
		motorIntakeArmRight.setInverted(true);
		motorIntakeRearLeft = motorRearLeft;
		motorIntakeRearRight = motorRearRight;
		motorIntakeRearRight.setInverted(true);
		ramDeploy1 = ram1In;
		ramDeploy2 = ram2In;
	}
	
	/**
	 * Manual method to retract Box Collector Arms.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Box Collector State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void armsRetract() {
		ramDeploy1.set(true);
		ramDeploy2.set(false);
	}
	
	/**
	 * Manual method to extend Box Collector Arms.
	 * <p>
	 * <b>Using this method directly
	 * bypasses the Box Collector State Machine which should be used in 
	 * Teleop.</b> If in Teleop, use <b>run()</b> instead!
	 * </p>
	 */
	public void armsExtend() {
		ramDeploy1.set(false);
		ramDeploy2.set(true);
	}
	
	public void armsStall() {
		ramDeploy1.set(true);
		ramDeploy2.set(true);
	}
	
	public boolean getArmsExtended() {
		if(ramDeploy1.get()) {
			return true;
		} else {
			return false;
		}
	}
	
	public void setArmMotorsSuckIn() {
		motorIntakeArmLeft.set(1);
		motorIntakeArmRight.set(1);
	}
	
	public void setArmMotorsSpitOut() {
		motorIntakeArmLeft.set(-1);
		motorIntakeArmRight.set(-1);
	}
	
	public void setArmMotorsSpinBox() {
		motorIntakeArmLeft.set(1);
		motorIntakeArmRight.set(-1);
	}
	
	public void setArmMotorsStop() {
		motorIntakeArmLeft.set(0);
		motorIntakeArmRight.set(0);
	}
	
	public void setRearMotorsSuckIn() {
		motorIntakeRearLeft.set(-1);
		motorIntakeRearRight.set(-1);
	}
	
	public void setRearMotorsSpitOut() {
		motorIntakeRearLeft.set(1);
		motorIntakeRearRight.set(1);
	}
	
	public void setRearMotorsSpinBox() {
		motorIntakeRearLeft.set(-1);
		motorIntakeRearRight.set(1);
	}
	
	public void setRearMotorsStop() {
		motorIntakeRearLeft.set(0);
		motorIntakeRearRight.set(0);
	}
}
