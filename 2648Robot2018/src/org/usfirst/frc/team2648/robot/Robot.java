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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
	private WPI_TalonSRX elefttop, eleftbot, erighttop, erightbot;
	private SpeedControllerGroup elevatorLeft, elevatorRight;
	private WPI_TalonSRX cleft,cright;
	private DigitalInput hallEffect;
	private PIDController drive,turn;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
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
		left.setDistancePerPulse(4*Math.PI/128);
		right.setDistancePerPulse(4*Math.PI/128);//setting the encoders pulse distance
		comp.start();//starts compressor
		e = new Encoder (8,9);//adds elevator encoder
		gyro = new ADXRS450_Gyro();//adds gyro
		
		elefttop = new WPI_TalonSRX(27);
		eleftbot = new WPI_TalonSRX(56); 
		erighttop = new WPI_TalonSRX(8);
		erightbot = new WPI_TalonSRX(2);
		
		cleft = new WPI_TalonSRX(59); 
		cright = new WPI_TalonSRX(12);
		
		hallEffect = new DigitalInput(4);
		
		drive = new PIDController(.45,.00,.001,left,new OutputDrive(rd, gyro));
		turn = new PIDController(.1,.00000,0,gyro,new OutputTurn(rd));
		
		//elevatorLeft = new SpeedControllerGroup(elefttop, eleftbot);
		//elevatorRight = new SpeedControllerGroup(erightbot, erighttop);
		
		SmartDashboard.putData("Left Encoder",left);
		SmartDashboard.putData("Right Encoder",right);//gives Data to the smart dash board
		SmartDashboard.putData("Elevator",e);
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
		//m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		left.reset();
		right.reset();
		gyro.reset();//resets the gyro and the encoders
		//System.out.println("Auto selected: " + m_autoSelected);
		drive.setSetpoint(60);
		drive.setPercentTolerance(5);
		drive.setOutputRange(-.75, .75);
		drive.enable();
		turn.setSetpoint(90);
		turn.setPercentTolerance(2);
		turn.setOutputRange(-.75, .75);
		//turn.enable();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	int c = 0;
	@Override
	public void autonomousPeriodic() {
		System.out.println(gyro.getAngle());
		//System.out.println();
		if(c == 0 && drive.isEnabled() && drive.getError()<2) {
			System.out.println("1");
			drive.disable();
			turn.enable();
			c++;
		}
		if(c == 1 && turn.isEnabled() && turn.getError()<5) {
			System.out.println("2");
			turn.disable();
			left.reset();
			drive.reset();
			drive.setSetpoint(60);
			drive.enable();
			c++;
		}
		if(c == 2 && drive.isEnabled() && drive.getError()<5) {
			System.out.println("3");
			drive.disable();
			turn.reset();
			turn.setSetpoint(-90);
			turn.enable();
			c++;
		}
		if(c == 3 && turn.isEnabled() && turn.getError()<5) {
			System.out.println(4);
			turn.disable();
			left.reset();
			drive.reset();
			drive.setSetpoint(60);
			drive.enable();
			c++;
		}
		if(c == 4 && drive.isEnabled() && drive.getError()<5) {
			System.out.println("3");
			drive.disable();
			cleft.set(-1);
			cright.set(1);
		}
		
		System.out.println("c " + c);
		
	}
	@Override
	public void  disabledInit() {
		super.disabledInit();
		drive.disable();
		c = 0;
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
			cleft.set(-1);
			cright.set(1);
		}
		else if (js2.getRawButton(5)) {
			ileft.set(-1);
			iright.set(1);
			cleft.set(1);
			cright.set(-1);
		}
		else {//if button 6 or 7 is not pressed the intake is at neutral
			ileft.set(0);
			iright.set(0);
			cleft.set(0);
			cright.set(0);
		}
		if (js2.getRawButton(2)) {//if button 0 is pressed intake numatics push out,if button 1 is pressed intake numatics pull in
			it.set(Value.kForward);
		}
		if (js2.getRawButton(1)) {
			it.set(Value.kReverse);//toggles intake numatics
		}
		if (js1.getRawButton(3)) {//toggles drive train when button 3 is pressed
			dt.set(Value.kForward);
		}
		if (js1.getRawButton(4)) {
			dt.set(Value.kReverse);
		}
		//System.out.println(left.getRate());
		//System.out.println(right.getRate());//gives encoder vaulues to drivers station
		/*if(Math.abs(left.getRate())<60) {
			//if(dt.get().equals(Value.kForward)) {
				dt.set(Value.kReverse);
			System.out.println("Shift");
		}
		if(Math.abs(left.getRate()) > 70) {
			if(dt.get().equals(Value.kReverse))
				dt.set(Value.kForward);
			System.out.println("Shift down");
		}*/
		System.out.println(left.getRate());
		//System.out.println(hallEffect.get());
		//if(js2.getRawButton(7))
			//e.reset();
		elevate(js2.getRawAxis(5));
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
	public void elevate(double output) {
		elefttop.set(-output);
		eleftbot.set(output);
		erighttop.set(-output);
		erightbot.set(-output); 
	}
	private class OutputDrive implements PIDOutput{

		private DifferentialDrive rd;
		private Gyro gyro;
		
		public OutputDrive(DifferentialDrive rd, Gyro gyro) {
			this.rd = rd;
			this.gyro = gyro;
		}
		
		@Override
		public void pidWrite(double output) {
			rd.arcadeDrive(output, -gyro.getAngle()/45);
			System.out.println(output);
		}
		
	}
	
	private class OutputTurn implements PIDOutput{

		private DifferentialDrive rd;
		
		public OutputTurn(DifferentialDrive rd) {
			this.rd = rd;
		}
		
		@Override
		public void pidWrite(double output) {
			rd.arcadeDrive(0, output);
			System.out.println(output);
		}
		
	}
}
