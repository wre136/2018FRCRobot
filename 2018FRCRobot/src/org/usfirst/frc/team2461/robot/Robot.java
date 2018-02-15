/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2461.robot;

import org.usfirst.frc.team2461.robot.autonomous.AutoCode;
import org.usfirst.frc.team2461.robot.autonomous.DriveForwardAuto;
import org.usfirst.frc.team2461.robot.autonomous.MoveLeftAuto;
import org.usfirst.frc.team2461.robot.autonomous.MoveRightAuto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String CenterAuto = "Robot Center";
	private static final String LeftAuto = "Robot Left";
	private static final String RightAuto = "Robot Right";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private DriverStation station = DriverStation.getInstance();
	
	private int motor_FL_Drive_int = 1;
	private int[] enc_FL_Drive_int = {0,1};
	private int motor_FL_Turn_int = 0;
	private int enc_FL_Turn_int = 0;
	
	private int motor_FR_Drive_int = 2;
	private int[] enc_FR_Drive_int = {2,3};
	private int motor_FR_Turn_int = 1;
	private int enc_FR_Turn_int = 1;
	
	private int motor_RL_Drive_int = 3;
	private int[] enc_RL_Drive_int = {4,5};
	private int motor_RL_Turn_int = 2;
	private int enc_RL_Turn_int = 2;
	
	private int motor_RR_Drive_int = 4;
	private int[] enc_RR_Drive_int = {6,7};
	private int motor_RR_Turn_int = 3;
	private int enc_RR_Turn_int = 3;
	
	private int motor_L_Arm_int = 4;
	private int motor_R_Arm_int = 5;
	private int motor_Box_Grabber_int = 6;
	private int[] armDoubleSolenoid = {0,1};
	
	private int motor_Box_Riser_int = 7;
	private int switch_Riser_Low_int = 8;
	private int switch_Riser_Mid_int = 9;
	private int switch_Riser_High_int = 10;
	
	private int motor_Robot_Lifter_1_int = 8;
	private int motor_Robot_Lifter_2_int = 9;
	private int robot_Lifter_Switch_Bottom_int = 11;
	private int robot_Lifter_Switch_Top_int = 12;
	
	
	MetalSkinsController player1 = new MetalSkinsController(0, true);
	
	//Create Drive train motors
	WPI_TalonSRX motor_FL_Drive = new WPI_TalonSRX( motor_FL_Drive_int);
	Encoder enc_FL_Drive = new Encoder(enc_FL_Drive_int[0], enc_FL_Drive_int[1]);
	Spark motor_FL_Turn = new Spark(motor_FL_Turn_int);
	MA3Encoder enc_FL_Turn = new MA3Encoder(enc_FL_Turn_int);
	SwerveMotor motorFL = new SwerveMotor(motor_FL_Drive, motor_FL_Turn, enc_FL_Turn, enc_FL_Drive);
	
	WPI_TalonSRX motor_FR_Drive = new WPI_TalonSRX(motor_FR_Drive_int);
	Encoder enc_FR_Drive = new Encoder(enc_FR_Drive_int[0], enc_FR_Drive_int[1]);
	Spark motor_FR_Turn = new Spark(motor_FR_Turn_int);
	MA3Encoder enc_FR_Turn = new MA3Encoder(enc_FR_Turn_int);
	SwerveMotor motorFR = new SwerveMotor(motor_FR_Drive, motor_FR_Turn, enc_FR_Turn, enc_FR_Drive);
	
	WPI_TalonSRX motor_RL_Drive = new WPI_TalonSRX(motor_RL_Drive_int);
	Encoder enc_RL_Drive = new Encoder(enc_RL_Drive_int[0], enc_RL_Drive_int[1]);
	Spark motor_RL_Turn = new Spark(motor_RL_Turn_int);
	MA3Encoder enc_RL_Turn = new MA3Encoder(enc_RL_Turn_int);
	SwerveMotor motorRL = new SwerveMotor(motor_RL_Drive, motor_RL_Turn, enc_RL_Turn, enc_RL_Drive);
	
	WPI_TalonSRX motor_RR_Drive = new WPI_TalonSRX(motor_RR_Drive_int);
	Encoder enc_RR_Drive = new Encoder(enc_RR_Drive_int[0], enc_RR_Drive_int[1]);
	Spark motor_RR_Turn = new Spark(motor_RR_Turn_int);
	MA3Encoder enc_RR_Turn = new MA3Encoder(enc_RR_Turn_int);
	SwerveMotor motorRR = new SwerveMotor(motor_RR_Drive, motor_RR_Turn, enc_RR_Turn, enc_RR_Drive);	
	
	SwerveDrive chassis = new SwerveDrive(motorFL, motorFR, motorRL, motorRR);	
	
	SwerveDriveAutoCommandFactory factory = SwerveDriveAutoCommandFactory.getInstance();
	
	Compressor c = new Compressor(0);
	
	Spark boxMotorGrabberL = new Spark(motor_L_Arm_int);
	Spark boxMotorGrabberR = new Spark(motor_R_Arm_int);
	Spark boxMotorGrabber = new Spark(motor_Box_Grabber_int);
	DoubleSolenoid armGrabber = new DoubleSolenoid(armDoubleSolenoid[0],armDoubleSolenoid[1]);
	BoxCollector boxCollector = new BoxCollector(boxMotorGrabberL, boxMotorGrabberR, boxMotorGrabber, armGrabber, player1);
	
	Spark riserMotor = new Spark(motor_Box_Riser_int);
	DigitalInput riserSwitchLow = new DigitalInput(switch_Riser_Low_int);
	DigitalInput riserSwitchMid = new DigitalInput(switch_Riser_Mid_int);
	DigitalInput riserSwitchHigh = new DigitalInput(switch_Riser_High_int);
	//BoxLifter boxLifter = new BoxLifter(riserMotor, riserSwitchLow, riserSwitchMid, riserSwitchHigh, player1);
	BoxLifter boxLifter = new BoxLifter(riserMotor, riserSwitchLow, riserSwitchHigh, player1);
	
	Talon lifterMotor1 = new Talon(motor_Robot_Lifter_1_int);
	Talon lifterMotor2 = new Talon(motor_Robot_Lifter_2_int);
	DigitalInput limitSwitchBottom = new DigitalInput(robot_Lifter_Switch_Bottom_int);
	DigitalInput limitSwitchTop = new DigitalInput(robot_Lifter_Switch_Top_int);
	RobotLift robotLift = new RobotLift(lifterMotor1, lifterMotor2, limitSwitchBottom, limitSwitchTop, player1);
	
	public static Timer timer = new Timer();
	
	/**
	 * Holds 3 letters {L or R} which tells what plates are your color
	 * Example: LRL says First Switch from you has your color on the left,
	 * Scale has right side as your color and the second switch from you has the
	 * left side as your color
	 */
	private String plateLayout;
	AutoCode autoCode;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@SuppressWarnings("static-access")
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("Robot Left Position", LeftAuto);
		m_chooser.addObject("Robot Center Position", CenterAuto);
		m_chooser.addObject("Robot Right Position", RightAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		chassis.setTurnPIDValues(0, 0.0345, 0.012, 0.0125);
		chassis.setTurnPIDValues(1, 0.0345, 0.012, 0.013);
		chassis.setTurnPIDValues(2, 0.0345, 0.014, 0.017);
		chassis.setTurnPIDValues(3, 0.0345, 0.012, 0.02);
		
		chassis.setDrivePIDValues(0, 0.04, 0.005, 0.001, 0);
		chassis.setDrivePIDValues(1, 0.04, 0.005, 0.001, 0);
		chassis.setDrivePIDValues(2, 0.04, 0.005, 0.001, 0);
		chassis.setDrivePIDValues(3, 0.04, 0.005, 0.001, 0);
		
		timer.start();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@SuppressWarnings("static-access")
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		
		plateLayout = station.getGameSpecificMessage();
		
		switch(m_autoSelected) {
			case LeftAuto:
				if(plateLayout.charAt(0) == 'L') {
					autoCode = new MoveRightAuto(chassis, boxCollector, boxLifter);
				} else {
					autoCode = new DriveForwardAuto(chassis);
				}
				break;
			case CenterAuto:
				if(plateLayout.charAt(0) == 'L') {
					autoCode = new MoveLeftAuto(chassis, boxCollector, boxLifter);
				} else if(plateLayout.charAt(0) == 'R') {
					autoCode = new MoveRightAuto(chassis, boxCollector, boxLifter);
				} else {
					autoCode = new DriveForwardAuto(chassis);
				}
				break;
			case RightAuto:
				if(plateLayout.charAt(0) == 'R') {
					autoCode = new MoveLeftAuto(chassis, boxCollector, boxLifter);
				} else {
					autoCode = new DriveForwardAuto(chassis);
				}
				break;
			default:
				break;
		}
		
		System.out.println("Auto selected: " + m_autoSelected);
		
		//Adding commands to make robot go in a rectangle
		/*chassis.addAutoCommand(factory.command_GoForward(60));
		chassis.addAutoCommand(factory.command_TurnLeft(90));
		chassis.addAutoCommand(factory.command_GoForward(90));
		chassis.addAutoCommand(factory.command_TurnLeft(90));
		chassis.addAutoCommand(factory.command_GoForward(60));
		chassis.addAutoCommand(factory.command_TurnLeft(90));
		chassis.addAutoCommand(factory.command_GoForward(0));
		chassis.addAutoCommand(factory.command_TurnLeft(90));
		chassis.addAutoCommand(factory.command_Stop());*/
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		autoCode.run();
		
		//chassis.driveAuto();
	}
	
	@Override
	public void teleopInit()
	{
		// TODO Auto-generated method stub
		super.teleopInit();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		chassis.drive(player1);
		boxCollector.run();
		boxLifter.run();
		robotLift.run();
	}

	@Override
	public void testInit() {
		motorFL.initTest();
		motorFR.initTest();
		motorRL.initTest();
		motorRR.initTest();
	}



	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		if(!motorFL.runTest()) {
			
		} else if(!motorFR.runTest()) {
			
		} else if(!motorRL.runTest()) {
			
		} else if(!motorRR.runTest()) {
			
		}
	}
	
	public void printDataToScreen()
	{
		SmartDashboard.putNumber("FLWheel Angle", chassis.getTurnEncoderAngles()[0]);
		SmartDashboard.putNumber("FRWheel Angle", chassis.getTurnEncoderAngles()[1]);
		SmartDashboard.putNumber("RLWheel Angle", chassis.getTurnEncoderAngles()[2]);
		SmartDashboard.putNumber("RRWheel Angle", chassis.getTurnEncoderAngles()[3]);
		
		SmartDashboard.putNumber("FLWheel Angle Setpoint", chassis.getTurnEncoderSetpoints()[0]);
		SmartDashboard.putNumber("FRWheel Angle Setpoint", chassis.getTurnEncoderSetpoints()[1]);
		SmartDashboard.putNumber("RLWheel Angle Setpoint", chassis.getTurnEncoderSetpoints()[2]);
		SmartDashboard.putNumber("RRWheel Angle Setpoint", chassis.getTurnEncoderSetpoints()[3]);
		
		SmartDashboard.putNumber("FLWheel Error", chassis.getPIDTurnErrors()[0]);
		SmartDashboard.putNumber("FRWheel Error", chassis.getPIDTurnErrors()[1]);
		SmartDashboard.putNumber("RLWheel Error", chassis.getPIDTurnErrors()[2]);
		SmartDashboard.putNumber("RRWheel Error", chassis.getPIDTurnErrors()[3]);
		
		SmartDashboard.putNumber("FLWheel Power", chassis.getDriveSpeed()[0]);
		SmartDashboard.putNumber("FRWheel Power", chassis.getDriveSpeed()[1]);
		SmartDashboard.putNumber("RLWheel Power", chassis.getDriveSpeed()[2]);
		SmartDashboard.putNumber("RRWheel Power", chassis.getDriveSpeed()[3]);
		
		SmartDashboard.putNumber("FLWheel Distance Traveled", chassis.getDistance()[0]);
		SmartDashboard.putNumber("FRWheel Distance Traveled", chassis.getDistance()[1]);
		SmartDashboard.putNumber("RLWheel Distance Traveled", chassis.getDistance()[2]);
		SmartDashboard.putNumber("RRWheel Distance Traveled", chassis.getDistance()[3]);
		
		SmartDashboard.putNumber("FLWheel Distance Setpoint", chassis.getDistanceSetpoints()[0]);
		SmartDashboard.putNumber("FRWheel Distance Setpoint", chassis.getDistanceSetpoints()[1]);
		SmartDashboard.putNumber("RLWheel Distance Setpoint", chassis.getDistanceSetpoints()[2]);
		SmartDashboard.putNumber("RRWheel Distance Setpoint", chassis.getDistanceSetpoints()[3]);
		
		SmartDashboard.putNumber("Speed", chassis.getDriveSpeed()[0]);
		SmartDashboard.putNumber("Command Direction", chassis.getCurrentCommand()[0]);
		SmartDashboard.putNumber("Command Distance", chassis.getCurrentCommand()[1]);
		SmartDashboard.putData(chassis.getTurnPID(0));
		SmartDashboard.putData(chassis.getTurnPID(1));
		SmartDashboard.putData(chassis.getTurnPID(2));
		SmartDashboard.putData(chassis.getTurnPID(3));
		
		SmartDashboard.putData(chassis.getDrivePID(0));
		SmartDashboard.putData(chassis.getDrivePID(1));
		SmartDashboard.putData(chassis.getDrivePID(2));
		SmartDashboard.putData(chassis.getDrivePID(3));
		
		SmartDashboard.putNumber("EncDriveCount", enc_FL_Drive.get());
		SmartDashboard.putNumber("Encoder Distance", enc_FL_Drive.getDistance());
		
		SmartDashboard.putString("Box Lifter State", boxLifter.getStateString());
		SmartDashboard.putString("Robot Lifter State", robotLift.getStateString());
	}
	
	@Override
	public void disabledPeriodic()
	{
		chassis.reset();
	}
	
	@Override
	public void disabledInit()
	{
		
	}
	
	@Override
	public void robotPeriodic()
	{
		printDataToScreen();
	}
}
