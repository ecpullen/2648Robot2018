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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.hal.PDPJNI;
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
	private static final String kLeftAuto = "Left";
	private static final String kRightAuto = "Right";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private Encoder left,right;// Encoder used for drive train
	private WPI_TalonSRX fleft,rleft,fright,rright;//drive train motor controlers
	private SpeedControllerGroup sleft,sright;//controler groups for drive train
	private DifferentialDrive rd;//drive train
	private XboxController js1,js2;//control joysticks
	private WPI_TalonSRX iright,ileft;//intake speed controlers
	private Compressor comp;//compressor
	private DoubleSolenoid dt,it;//intake and drive train numatics
	private boolean high;//tells us if shifters are in high gear
	private Encoder e; //elevater encoder
	private ADXRS450_Gyro gyro;//gyro
	private double pSpeed = 0;
	private WPI_TalonSRX elefttop, eleftbot, erighttop, erightbot;
	private WPI_TalonSRX cleft,cright;
	private DigitalInput hallEffect, ebottom, etop;
	private PIDController drive,turn;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("Left", kLeftAuto);
		m_chooser.addObject("Right", kRightAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		fleft = new WPI_TalonSRX (60);
		rleft = new WPI_TalonSRX (11);
		fright = new WPI_TalonSRX (25);
		rright = new WPI_TalonSRX (32);//creates all 4 drive train motors
		sleft = new SpeedControllerGroup (fleft,rleft);
		sright = new SpeedControllerGroup (fright,rright);//makes the speed controller groups
		rd = new DifferentialDrive (sleft,sright);//creating drive train
		js1 = new XboxController(0);
		js2 = new XboxController(1);//creating joysticks
		iright = new WPI_TalonSRX (45);
		ileft = new WPI_TalonSRX (28);//creates the 2 intake motor controllers
		comp = new Compressor ();//creates the compressor
		dt = new DoubleSolenoid (0,1);
		it = new DoubleSolenoid (2,3);//creates the double solenoids
		high = false;// set high
		
		left = new Encoder (2,3);
		right = new Encoder (0,1);//creates encoders
		left.setDistancePerPulse(4*Math.PI/128);
		right.setDistancePerPulse(4*Math.PI/128);//setting the encoders pulse distance
		comp.start();//starts compressor
		e = new Encoder (8,9);//adds elevator encoder
		gyro = new ADXRS450_Gyro();//adds gyro
		e.setDistancePerPulse(.0175);
		e.setReverseDirection(false);
		elefttop = new WPI_TalonSRX(27);
		eleftbot = new WPI_TalonSRX(56); 
		erighttop = new WPI_TalonSRX(8);
		erightbot = new WPI_TalonSRX(2);
		
		cleft = new WPI_TalonSRX(59); 
		cright = new WPI_TalonSRX(12);
		
		hallEffect = new DigitalInput(4);
		ebottom = new DigitalInput(5);
		etop = new DigitalInput(6);
		
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
	String key;
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto)
		left.reset();
		right.reset();
		gyro.reset();//resets the gyro and the encoders
		//System.out.println("Auto selected: " + m_autoSelected);
		
		drive.setPercentTolerance(5);
		drive.setOutputRange(-1, 1);
		drive.enable();
		turn.setPercentTolerance(2);
		turn.setOutputRange(-.75, .75);
		//turn.enable();
		dt.set(Value.kForward);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	int c = 0;
	int c2 = 0;
	@Override
	public void autonomousPeriodic() {
		if(key.length() != 3) {
			key = DriverStation.getInstance().getGameSpecificMessage();
		}
		else {
			if(m_autoSelected.equals(kRightAuto)) {//selects which auto the robot does
				autoRight(key);
			}
			if(m_autoSelected.equals(kLeftAuto)) {//selects which auto the robot does
				autoLeft(key);
			}
			if(m_autoSelected.equals(kDefaultAuto)) {
				autoDefault(key);
			}
		}
	}
	
	public void autoLeft(String s) {
		System.out.println(s);
		if(s.charAt(1)=='l') {
			autoscaleRight(false);
			System.out.println("Scale");
		}
		else if (s.charAt(2)=='l') {
			autoSwitchRight(false);
			System.out.println("Switch");
		}
		else {
			autolineDrive();
			System.out.println("AutoLineDrive");
		}
	}
	
	public void autoscaleRight(boolean right) {
		int i = right ? 1 : -1;
		System.out.println("c "+c);
		if(c == 0) {
			dt.set(Value.kReverse);// sets motors to high gear
			drive.reset();//resets drive pids
			left.reset();
			drive.setSetpoint(450);//drives robot forward 450 inches
			drive.enable();//sets drive pid
			c++;//increases c by 1
		}
		if(c == 1 && drive.isEnabled() && drive.getError()<2) {
			drive.disable();
			turn.setSetpoint(-i*30);//turns robot after going forwards
			turn.enable();//sets turn pid
			c++;//increases c by 1
		}
		if(c==2&&Math.abs(turn.getError())<5) {
			turn.disable();
		}
		if(c  == 2&&etop.get()&&!turn.isEnabled()) {// if c = 2 and the elevator is not at the top it moves the elevator up
			elevate(-1);//moves elevator up
			it.set(Value.kReverse);//toggles intake numatics
		}
		if(c == 2&&!etop.get()) {//if c = 2 and elevator is not at the top it does not move the elevator
			elevate(0);// does not move elevator
			c++;
		}
		if (c == 3) {//if c = 3 the claw pushes out the cube
			cleft.set(-1);
			cright.set(1);//makes claw go out
			c2++;
		}
		if(c==3&&c2==60) {
			cleft.set(0);//resets claw
			cright.set(0);
			elevate(1);//brings elevator back down
			c++;
		}
		if(c==4&&!ebottom.get()) {//if c=4 and elevator is not on the bottom then the elevator does not move
			elevate(0);
			c++;
		}
		if(c==5) {
			gyro.reset();//resets gyro
			turn.reset();
			turn.setSetpoint(-i*110);//turns robot
			turn.enable();
			c++;
		
		}
		if(c==6&&turn.getError()<5) {
			turn.disable();
			c++;
		}
	}
	public void autoSwitchRight(boolean right) {
		int i = right ? 1 : -1;
		if(c == 0) {
			dt.set(Value.kReverse);// sets motors to high gear
			drive.reset();//resets drive pins
			left.reset();
			drive.setSetpoint(170);//drives robot forward 160 inches
			drive.enable();//sets turn pin
			c++;//increases c by 1
		}
		if(c == 1 && drive.isEnabled() && drive.getError()<2) {
			drive.disable();
			turn.setSetpoint(-i*30);//turns robot after going forwards
			turn.enable();//sets turn pin
			c++;//increases c by 1
		}
		if(c==2&&Math.abs(turn.getError())<5) {
			turn.disable();
		}
		if (c == 2&&!turn.isEnabled()) {//if c = 2 the claw pushes out the cube
			cleft.set(-1);
			cright.set(1);//makes claw go out
			c2++;
		}
		if(c==2&&c2==60) {
			cleft.set(0);//resets claw
			cright.set(0);
			elevate(1);//brings elevator back down
			c++;
		}
		if(c==3&&!ebottom.get()) {//if c=3 and elevator is on the bottom then the elevator does not move
			elevate(0);
			c++;
		}
	}
	public void autoRight (String s) {//s tells us which scales are ours
		System.out.println(s);
		if(s.charAt(1)=='r') {
			autoscaleRight(true);
			System.out.println("Scale");
		}
		else if (s.charAt(2)=='r') {
			autoSwitchRight(true);
			System.out.println("Switch");
		}
		else {
			autolineDrive();
			System.out.println("AutoLineDrive");
		}
	}
	public void autolineDrive() {
		if(c == 0) {
			drive.setSetpoint(30);
			c++;
		}
		if(c == 1 && drive.isEnabled() && drive.getError()<5) {
			System.out.println("1");
			drive.disable();
			it.set(Value.kReverse);
			elevate(1);
			c++;
		}
		if(c == 2 && !ebottom.get()) {
			elevate(0);
		}
		
	}
	public void autoDefault(String key) {
		int i = key.charAt(0)=='r' ? 1 : -1;       
		if(c == 0) {
			drive.setSetpoint(30);
			c++;
		}
		if(c == 1 && drive.isEnabled() && drive.getError()<5) {
			System.out.println("1");
			drive.disable();
			turn.setSetpoint(i*60);
			turn.enable();
			c++;
		}
		if(c == 2 && turn.isEnabled() && turn.getError()<5) {
			System.out.println("2");
			turn.disable();
			gyro.reset();
			left.reset();
			drive.reset();
			drive.setSetpoint(66);
			drive.enable();
			c++;
		}
		if(c == 3 && drive.isEnabled() && drive.getError()<2) {
			System.out.println("3");
			drive.disable();
			gyro.reset();
			turn.reset();
			turn.setSetpoint(-i*60);
			turn.enable();
			c++;
		}
		if(c == 4 && turn.isEnabled() && Math.abs(turn.getError())<5) {
			System.out.println(4);
			turn.disable();
			gyro.reset();
			left.reset();
			drive.reset();
			drive.setSetpoint(56);
			drive.enable();
			c++;
		}
		if(c == 5 && drive.isEnabled() && drive.getError()<2) {
			System.out.println("3");
			gyro.reset();
			drive.disable();
			
		}
		if(c == 5) {
			if(c2 < 60)
				c2++;
			else {
				c2 = 0;
				cleft.set(-.75);
				cright.set(.75);
				c++;
			}
		}
		if(c == 6 && c2 > 60) {
			cleft.set(0);
			cright.set(0);
			c++;
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
		if (js2.getRawButton(6) || js1.getRawButton(6)) {//if button 6 is pressed the intake pulls in, 5 is pressed intake pushes out
			ileft.set(.85);
			iright.set(-.85);
			cleft.set(-1);
			cright.set(1);
		}
		else if (js2.getRawButton(5) || js1.getRawButton(5)) {
			ileft.set(-.85);
			iright.set(.85);
			cleft.set(.8);
			cright.set(-.8);
		}
		else if(js2.getRawButton(4)) {
			cleft.set(-.6);
			cright.set(.6);
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
		
		//if(PDPJNI.getPDPTotalCurrent(7)>10)
		//	js2.setRumble(RumbleType.kLeftRumble, .25);
		//System.out.println(etop.get() +" "+ ebottom.get());
		//System.out.println(e.getDistance());
		if(etop.get() && (js2.getRawAxis(5)<-.3))
			elevate(-1);
		else if(ebottom.get() && (js2.getRawAxis(5)>.3))
			elevate(1);
		else
			elevate(0);
		
		if(!ebottom.get())
			e.reset();
		

		System.out.println("rate " + e.getRate());
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
			rd.arcadeDrive(-pSpeed, js1.getX(Hand.kLeft)*.85);
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
			
			rd.arcadeDrive(pSpeed, js1.getX(Hand.kLeft)*.85);
		}
	}
	
	private double ePSpeed = 0;//stores the previous power of the elevator.
	private static final double kRAMP = .05; //increase this to speed up the elevator.
	
	public void elevate(double output) {
		if(output == 0) {
			setElevate(0);
			return;
		}
		if(ePSpeed*output<0) {
			ePSpeed = output/100;
		}
		if(ePSpeed > 0) {
			if(Math.max(ePSpeed, output) == ePSpeed) {
				ePSpeed = output;
			}
			else {
				ePSpeed += kRAMP;
			}
		}
		else {
			if(Math.min(ePSpeed, output) == ePSpeed) {
				ePSpeed = output;
			}
			else {	
				ePSpeed -= kRAMP;
			}
		}
		if(ePSpeed>1)
			ePSpeed = 1;
		if(ePSpeed<-1)
			ePSpeed = -1;
		
		//setElevate(ePSpeed);
		setElevate(ePSpeed);
	}
	
	private double pScale = 0;//amount scaled down by.
	private static final double kACCEL = 2.5;//calculate as (max_velocity - final_velocity)/(stop_distance)
	
	public void decel(double output) {
		if(e.getDistance()>72 && output > 0) {
			if(e.getRate() > 40-kACCEL*(e.getDistance()-72)) {
				pScale -= .1;
				
			}
			System.out.println("pScale " + pScale);
		}
		if(e.getDistance()<12 && output < 0) {
			if(e.getRate() < -40+kACCEL*(12-e.getDistance())) {
				pScale += .1;
			}
		}
		//System.out.println(pScale);
		setElevate(output + pScale);
	}
	
	public void setElevate(double output) {
		elefttop.set(output);
		eleftbot.set(-output);
		//erighttop.set(-output);
		//erightbot.set(-output); 
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
			rd.arcadeDrive(output, -.3-gyro.getAngle()/10);
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
