package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;

public class RobotLift
{
	private SpeedController[] motor = new SpeedController[2];
	private DigitalInput limitSwitchBottom;
	private DigitalInput limitSwitchTop;
	private MetalSkinsController player;
	
	private enum State {
		BEGIN, LOW, LOWERING, HIGH, LIFTING, IDLE
	}
	
	private State stateNow;
	
	/**
	 * @param motor1 Motor 1
	 * @param motor2 Motor 2
	 * @param limistSwitchBottomIn Limit switch for the bottom
	 * @param limitSwitchTopIn Limit switch for the top
	 * @param playerIn MetalSkinsController to control the robot lift
	 */
	public RobotLift(SpeedController motor1, SpeedController motor2, DigitalInput limistSwitchBottomIn, DigitalInput limitSwitchTopIn, MetalSkinsController playerIn)
	{
		motor[0] = motor1;
		motor[1] = motor2;
		limitSwitchBottom = limistSwitchBottomIn;
		limitSwitchTop = limitSwitchTopIn;
		player = playerIn;
		stateNow = State.BEGIN;
		setMotor(0);
	}
	
	public void run() {
		switch(stateNow) {
			case BEGIN:
				begin();
				break;
			case HIGH:
				high();
				break;
			case IDLE:
				idle();
				break;
			case LIFTING:
				lifting();
				break;
			case LOW:
				low();
				break;
			case LOWERING:
				lowering();
				break;
			default:
				break;
			
		}
	}
	
	private void begin() {
		setMotor(0);
		if(!limitSwitchBottom.get()) {
			stateNow = State.LOW;
		} else if(!limitSwitchTop.get()) {
			stateNow = State.HIGH;
		} else {
			stateNow = State.IDLE;
		}
	}
	
	private void idle() {
		if(player.getYButton() && player.getXButton()) {
			return;
		} else if(player.getYButton()) {
			setMotor(1);
			stateNow = State.LIFTING;
		} else if(player.getXButton()) {
			setMotor(-1);
			stateNow = State.LOWERING;
		}
	}
	
	private void lifting() {
		if(limitSwitchTop.get()) {
			if(!player.getYButton()) {
				setMotor(0);
				stateNow = State.IDLE;
			}
		} else {
			setMotor(0);
			stateNow = State.HIGH;
		}
	}
	
	private void lowering() {
		if(limitSwitchBottom.get()) {
			if(!player.getXButton()) {
				setMotor(0);
				stateNow = State.IDLE;
			}
		} else {
			setMotor(0);
			stateNow = State.LOW;
		}
	}
	
	private void high() {
		if(player.getXButton()) {
			setMotor(-1);
			stateNow = State.LOWERING;
		}
	}
	
	private void low() {
		if(player.getYButton()) {
			setMotor(1);
			stateNow = State.LIFTING;
		}
	}
	
	private void setMotor(double value) {
		motor[0].set(value);
		motor[1].set(value);
	}
	
	public State getState() {
		return stateNow;
	}
	
	public String getStateString() {
		switch(stateNow) {
			case BEGIN:
				return "BEGIN";
			case HIGH:
				return "HIGH";
			case IDLE:
				return "IDLE";
			case LIFTING:
				return "LIFTING";
			case LOW:
				return "LOW";
			case LOWERING:
				return "LOWERING";
			default:
				return "NULL";
		}
	}
}
