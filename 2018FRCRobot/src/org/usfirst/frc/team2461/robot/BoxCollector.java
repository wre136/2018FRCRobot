package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class BoxCollector
{
	private SpeedController motorIntakeLeft;
	private SpeedController motorIntakeRight;
	private SpeedController motorIntakeRear;
	private DoubleSolenoid ramDeploy;
	private MetalSkinsController player;
	
	private enum State {
		BEGIN, DEPLOYING, RETRACT, REST, SUCK_IN, SPIT_OUT
	}
	
	private State stateNow;
	
	/**
	 * @param motorLeft Left motor on arm
	 * @param motorRight Right motor on arm
	 * @param motorRear Motor to run wheels inside the box
	 * @param ramIn DoubleSolenoid that open-closes the arms
	 * @param playerIn MetalSkinController to be used to activate the box collector
	 */
	public BoxCollector(SpeedController motorLeft, SpeedController motorRight, SpeedController motorRear, DoubleSolenoid ramIn, MetalSkinsController playerIn) {
		motorIntakeLeft = motorLeft;
		motorIntakeRight = motorRight;
		motorIntakeRight.setInverted(true);
		motorIntakeRear = motorRear;
		ramDeploy = ramIn;
		player = playerIn;
		stateNow = State.BEGIN;
	}
	
	public void run() {
		switch(stateNow) {
			case BEGIN:
				begin();
				break;
			case DEPLOYING:
				deploying();
				break;
			case REST:
				rest();
				break;
			case RETRACT:
				retract();
				break;
			case SPIT_OUT:
				spitOut();
				break;
			case SUCK_IN:
				suckIn();
				break;
			default:
				break;
		}
	}
	
	private void begin()
	{
		ramDeploy.set(DoubleSolenoid.Value.kForward);
		setMotorSpeed(0);
		stateNow = State.REST;
	}
	
	private void deploying() {
		ramDeploy.set(DoubleSolenoid.Value.kReverse);
		stateNow = State.REST;
	}
	
	private void rest() {
		if(player.getTriggerAxis(Hand.kRight) == 1 && player.getTriggerAxis(Hand.kLeft) == 1) {
			return;
		} else if(player.getTriggerAxis(Hand.kRight) == 1) {
			setMotorSpeed(1);
			stateNow = State.SUCK_IN;
		} else if(player.getTriggerAxis(Hand.kLeft) == 1) {
			setMotorSpeed(-1);
			stateNow = State.SPIT_OUT;
		} else if(player.getAButton()){
			ramDeploy.set(DoubleSolenoid.Value.kForward);
			stateNow = State.RETRACT;
		}
	}
	
	private void suckIn() {
		if(player.getTriggerAxis(Hand.kRight) == 1) {
			return;
		} else {
			setMotorSpeed(0);
			stateNow = State.REST;
		}
	}
	
	private void spitOut() {
		if(player.getTriggerAxis(Hand.kLeft) == 1) {
			return;
		} else {
			setMotorSpeed(0);
			stateNow = State.REST;
		}
	}
	
	private void retract() {
		if(player.getAButton()) {
			return;
		} else {
			ramDeploy.set(DoubleSolenoid.Value.kReverse);
			stateNow = State.REST;
		}
	}
	
	private void setMotorSpeed(double speed) {
		motorIntakeLeft.set(speed);
		motorIntakeRight.set(speed);
		motorIntakeRear.set(speed);
	}
	
	public State getState() {
		return stateNow;
	}
	
	public String getStateString() {
		switch(stateNow) {
			case BEGIN:
				return "BEGIN";
			case DEPLOYING:
				return "DEPLOYING";
			case REST:
				return "REST";
			case RETRACT:
				return "RETRACT";
			case SPIT_OUT:
				return "SPIT_OUT";
			case SUCK_IN:
				return "SUCK_IN";
			default:
				return "NULL";
		}
	}
	
	public void armsRetract() {
		ramDeploy.set(DoubleSolenoid.Value.kReverse);
		stateNow = State.REST;
	}
	
	public void armsExtend() {
		ramDeploy.set(DoubleSolenoid.Value.kForward);
		stateNow = State.REST;
	}
	
	public void spitBoxOut() {
		setMotorSpeed(1);
	}
	
	public void suckBoxIn() {
		setMotorSpeed(-1);
	}
	
	public void stopBoxSucker() {
		setMotorSpeed(0);
	}
}
