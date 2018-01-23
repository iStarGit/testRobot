/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */  
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// TESTING
package org.usfirst.frc.team3035.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions correspondin to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	// private static final String kMilesAuto = "Guess Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	String gameData; // returns L or R in a string of 3 chars, in order corresponding to your alliance
	Compressor compressor = new Compressor(0);
	Solenoid exSolo = new Solenoid(1);
	DoubleSolenoid exDbl = new DoubleSolenoid(1,2);
	Spark LF = new Spark(0), LB = new Spark(1), RF = new Spark(2), RB = new Spark(3); 
	
	Timer timer; 
	
	Joystick player1 = new Joystick(1), player2 = new Joystick (2); 
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
	  //m_chooser.addObject("Guess Auto", kMilesAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
	
		
		gameData =  DriverStation.getInstance().getGameSpecificMessage(); // to test, go onto	
		// driver station software and enter game datal
		
		if (gameData.charAt(0) == 'L') // or 'R'; 1 for scale, 2 for opposing switch
		{
			
		}
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
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
				/*
				 * case kMilesAuto:
				 * timer.reset();
				 * timer.start();
				 * while (timer.get() < 4.5) {
				 * LF.set(-.75);
				 * LB.set(-.75);
				 * RF.set(.75);
				 * RB.set(.75);
				 * }
				 * break;
				 */
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		//simple drivetrain
		
		double left = (player1.getRawAxis(1) * -1); // *-1 to inverse left side
		double right = player1.getRawAxis(5);
		
		LF.set(left);
		LB.set(left);
		RF.set(right);
		RB.set(right);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
