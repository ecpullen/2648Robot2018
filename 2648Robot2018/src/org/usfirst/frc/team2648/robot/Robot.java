/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2648.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private Encoder left,right;// Encoder used for drive train
	private WPI_TalonSRX fleft,rleft,fright,rright;//drive train motor controlers
	private SpeedControllerGroup sleft,sright;//controler groups for drive train
	private DifferentialDrive rd;//drive train
	private Joystick js1,js2;//control joysticks
	private WPI_TalonSRX iright,ileft;//intake speed controlers
	private Compressor comp;//compressor
	private DoubleSolenoid dt,it;//intake and drive train numatics
	private boolean high;//tells us if shifters are in high gear
	private Encoder e; //elevater encoder
	private ADXRS450_Gyro gyro;//gyro
	private double pSpeed = 0;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		fleft = new WPI_TalonSRX (60);
		rleft = new WPI_TalonSRX (11);
		fright = new WPI_TalonSRX (25);
		rright = new WPI_TalonSRX (32);//creates all 4 drive train motors
		sleft = new SpeedControllerGroup (fleft,rleft);
		sright = new SpeedControllerGroup (fright,rright);//makes the speed controler groups
		rd = new DifferentialDrive (sleft,sright);//creating drive train
		js1 = new Joystick (0);
		js2 = new Joystick (1);//creating joysticks
		iright = new WPI_TalonSRX (45);
		ileft = new WPI_TalonSRX (28);//creates the 2 intake motor controlers
		comp = new Compressor ();//creates the compressor
		dt = new DoubleSolenoid (0,1);
		it = new DoubleSolenoid (2,3);//creates the doulbe solenoids
		high = false;// set high
		
		left = new Encoder (2,3);
		right = new Encoder (0,1);//creates encoders
		left.setDistancePerPulse(1);
		right.setDistancePerPulse(1);//setting the encoders pulse distance
		comp.start();//starts compressor
		e = new Encoder (4,5);//adds elevater encoder
		gyro = new ADXRS450_Gyro();//adds gyro
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
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		left.reset();
		right.reset();
		gyro.reset();//resets the gyro and the encoders
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		if(left.getDistance()<60) {
			double term = gyro.getAngle();//tells us how far off we are from the path
			rd.arcadeDrive(.5, 0);
			
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		//rd.arcadeDrive(-js1.getY(Hand.kLeft), js1.getX(Hand.kLeft));// lets opperator control the drive train
		arcadeDrive();
		if (js2.getRawButton(6)) {//if button 6 is pressed the intake pulles in, 7 is pressed intake pushes out
			ileft.set(1);
			iright.set(-1);
		}
		else if (js2.getRawButton(7)) {
			ileft.set(-1);
			iright.set(1);
		}
		else {//if button 6 or 7 is not pressed the intake is at nuetral
			ileft.set(0);
			iright.set(0);
		}
		if (js2.getRawButton(0)) {//if button 0 is pressed intake numatics push out,if button 1 is pressed intake numatics pull in
			it.set(Value.kForward);
		}
		if (js2.getRawButton(1)) {
			it.set(Value.kReverse);//toggles intake numatics
		}
		if (js1.getRawButton(3)) {//toggles drive train when button 3 is pressed
			dt.set(Value.kForward);
		}
		if (js1.getRawButton(2)) {
			dt.set(Value.kReverse);
		}
		System.out.println(left.getDistance());
		System.out.println(right.getDistance());//gives encoder vaulues to drivers station
		SmartDashboard.putData(left);
		SmartDashboard.putData(right);//gives Data to the smart dash board
	}

	/** 
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	private final double minV = .40;
	public void arcadeDrive() {
		double pow = js1.getY(Hand.kLeft);
		if(pow<=0) {
			if(pow != 0) {
				if(pSpeed == 0) {
					pSpeed = minV*.5 * pow < 0 ? 1 : -1;
				}
				else {
					pSpeed += .005 * pow < 0 ? 1 : -1;
				}
			pSpeed = Math.min(pSpeed, pow);
			}
			else
				pSpeed = 0;
			rd.arcadeDrive(-pSpeed, js1.getX(Hand.kRight)*.75);
		}
		else {
			pow *= -1;
			if(pow != 0) {
				if(pSpeed == 0) {
					pSpeed = minV * pow < 0 ? 1 : -1;
				}
				else {
					pSpeed += .005 * pow < 0 ? 1 : -1;
				}
			pSpeed = Math.min(pSpeed, pow);
			}
			else
				pSpeed = 0;
			rd.arcadeDrive(pSpeed, js1.getX(Hand.kRight)*.75);
		}
	}
}
