/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2461.robot;

import java.util.LinkedList;

import org.usfirst.frc.team2461.robot.autonomous.AutoCode;
import org.usfirst.frc.team2461.robot.autonomous.DriveForwardAuto;
import org.usfirst.frc.team2461.robot.autonomous.DriveForwardAutoBasic;
import org.usfirst.frc.team2461.robot.autonomous.MoveLeftAuto;
import org.usfirst.frc.team2461.robot.autonomous.MoveRightAuto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
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
	private int motor_Box_Grabber_Rear1_int = 6;
	private int motor_Box_Grabber_Rear2_int = 7;
	private int[] armDoubleSolenoid = {0,1};
	
	private int motor_Box_Riser_int = 8;
	private int switch_Riser_Low_int = 8;
	private int switch_Riser_Mid_int = 9;
	private int switch_Riser_High_int = 11;
	
	private int motor_Robot_Lifter_1_int = 9;
	private int motor_Robot_Lifter_2_int = 10;
	
	
	MetalSkinsController player1 = new MetalSkinsController(0, true);
	MetalSkinsController player2 = new MetalSkinsController(1, true);
	
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
	
	Spark riserMotor = new Spark(motor_Box_Riser_int);
	DigitalInput riserSwitchLow = new DigitalInput(switch_Riser_Low_int);
	DigitalInput riserSwitchMid = new DigitalInput(switch_Riser_Mid_int);
	DigitalInput riserSwitchHigh = new DigitalInput(switch_Riser_High_int);
	BoxLifter boxLifter = new BoxLifter(riserMotor, riserSwitchLow, riserSwitchMid, riserSwitchHigh);
	//BoxLifter boxLifter = new BoxLifter(riserMotor, riserSwitchLow, riserSwitchHigh, player1);
	
	Spark boxMotorGrabberL = new Spark(motor_L_Arm_int);
	Spark boxMotorGrabberR = new Spark(motor_R_Arm_int);
	Spark boxMotorGrabberRear1 = new Spark(motor_Box_Grabber_Rear1_int);
	Spark boxMotorGrabberRear2 = new Spark(motor_Box_Grabber_Rear2_int);
	//DoubleSolenoid armGrabber = new DoubleSolenoid(armDoubleSolenoid[0],armDoubleSolenoid[1]);
	Solenoid armGrabber1 = new Solenoid(armDoubleSolenoid[0]);
	Solenoid armGrabber2 = new Solenoid(armDoubleSolenoid[1]);
	BoxCollector boxCollector = new BoxCollector(boxMotorGrabberL, boxMotorGrabberR, boxMotorGrabberRear1, boxMotorGrabberRear2, armGrabber1, armGrabber2);
	
	BoxManager boxManager = new BoxManager(boxCollector, boxLifter, player2);
	
	Talon lifterMotor1 = new Talon(motor_Robot_Lifter_1_int);
	Talon lifterMotor2 = new Talon(motor_Robot_Lifter_2_int);
	RobotLift robotLift = new RobotLift(lifterMotor1, lifterMotor2, player2);
	
	CameraServer camServer = CameraServer.getInstance();
	
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
		
		chassis.setDrivePIDValues(0, 0.1, 0.005, 0.001, 0);
		chassis.setDrivePIDValues(1, 0.1, 0.005, 0.001, 0);
		chassis.setDrivePIDValues(2, 0.1, 0.005, 0.001, 0);
		chassis.setDrivePIDValues(3, 0.1, 0.005, 0.001, 0);
		
		c.setClosedLoopControl(true);
		c.start();
		
		boxManager.setBoxLifterModeAutomatic(false);
		motor_RR_Drive.setInverted(true);
		boxManager.boxCollectorArmRetract();
		timer.start();
		camServer.startAutomaticCapture();
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
			case kDefaultAuto:
				autoCode = new DriveForwardAutoBasic(chassis, boxManager);
				break;
			case LeftAuto:
				if(plateLayout.charAt(0) == 'L') {
					autoCode = new MoveRightAuto(chassis, boxManager);
				} else {
					autoCode = new DriveForwardAutoBasic(chassis, boxManager);
				}
				break;
			case CenterAuto:
				if(plateLayout.charAt(0) == 'L') {
					autoCode = new MoveLeftAuto(chassis, boxManager);
				} else if(plateLayout.charAt(0) == 'R') {
					autoCode = new MoveRightAuto(chassis, boxManager);
				} else {
					autoCode = new DriveForwardAuto(chassis);
				}
				break;
			case RightAuto:
				if(plateLayout.charAt(0) == 'R') {
					autoCode = new MoveLeftAuto(chassis, boxManager);
				} else {
					autoCode = new DriveForwardAutoBasic(chassis, boxManager);
				}
				break;
			default:
				break;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		autoCode.run();
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
		boxManager.run();
		robotLift.run();
	}

	@Override
	public void testInit() {
		motorFL.initTest();
		motorFR.initTest();
		motorRL.initTest();
		motorRR.initTest();
		boxManager.initTest();
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
			
		} else if(!boxManager.runTest()) {
			
		}
	}
	
	public void printDataToScreen()
	{
//		chassis.debugWheel(0);
//		chassis.debugWheel(1);
//		chassis.debugWheel(2);
//		chassis.debugWheel(3);
//		
//		boxManager.debug(0);
//		boxManager.debug(1);
//		boxManager.debug(2);
		
//		autoCode.debug();
	}
	
	@Override
	public void disabledPeriodic()
	{
		
	}
	
	@Override
	public void disabledInit()
	{
		chassis.reset();
		boxManager.reset();
		robotLift.reset();
		if(autoCode != null)
			autoCode.reset();
	}
	
	@SuppressWarnings("unused")
	@Override
	public void robotPeriodic()
	{
		if(true)
			printDataToScreen();
	}
}
