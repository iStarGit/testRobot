/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */  
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// TESTING
package org.usfirst.frc.team3035.robot;

import org.opencv.video.KalmanFilter;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions correspondin to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot implements PIDOutput
{
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private SendableChooser<String> m_chooser = new SendableChooser<>();
    
    String gameData; // returns L or R in a string of 3 chars, in order corresponding to your alliance
    AHRS ahrs;
    
    //Spark LF, LB, RF, RB; ACTUAL
    Spark LF, LB;
    Victor RF, RB;
    Spark iLF, iLB, iRF, iRB;
    Spark lift;
    
    Timer timer; 
    PIDController turnController;
    double rotateToAngleRate;
    
    Joystick player1 = new Joystick(1), player2 = new Joystick (2); 
    
    static final double kP = 0.175;
    static final double kI = 0.01;
    static final double kD = 0.115;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;    
    
    static final double kTargetAngleDegrees = -90.0f;
    
    double target;
    /*
    Compressor compressor;
    DoubleSolenoid exSoloIn;
    DoubleSolenoid exSoloIn2;
     */
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @SuppressWarnings("deprecation")
    @Override
    public void robotInit() {
        m_chooser.addDefault("Default Auto", kDefaultAuto);
        m_chooser.addObject("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        /*
        LF= new Spark(4);
        LB = new Spark(3); 
        RF = new Spark(2);
        RB = new Spark(1); 
        ACTUAL
        */
        
        LF = new Spark(2);
        LB = new Spark(3);
        RF = new Victor(1);
        RB = new Victor(0);
        
        iLF= new Spark(8);
        iLB = new Spark(7); 
        iRF = new Spark(9);
        iRB = new Spark(6);
        lift = new Spark(5);
        
        timer = new Timer();
        /*
        compressor = new Compressor(0);
        
        exSoloIn = new DoubleSolenoid(1, 0);
        exSoloIn2 = new DoubleSolenoid(2, 3);
        
        compressor.start();
        */
        
        gameData =  DriverStation.getInstance().getGameSpecificMessage(); // to test, go onto    
        // driver station software and enter game datal
        
        /*if (gameData.charAt(0) == 'L') // or 'R'; 1 for scale, 2 for opposing switch
        {
            
        }*/
        
            
    try {
                /***********************************************************************
                 * navX-MXP:
                 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
                 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
                 * 
                 * navX-Micro:
                 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
                 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
                 * 
                 * Multiple navX-model devices on a single robot are supported.
                 ************************************************************************/
                ahrs = new AHRS(Port.kMXP); 
            } catch (RuntimeException ex ) {
                DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    
    turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    turnController.setInputRange(-180.0f,  180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
    turnController.disable();
    
    /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
    /* tuning of the Turn Controller's P, I and D coefficients.            */
    /* Typically, only the P value needs to be modified.                   */
    LiveWindow.addActuator("DriveSystem", "RotateController", turnController);    
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

        turnController.disable();
        timer.reset();
        timer.start();
        ahrs.zeroYaw();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        
        /*switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
                */
        
            //if(timer.get() < 2.0) 
            //{
                
            /*    
                if(!turnController.isEnabled())
                {
                turnController.setSetpoint(-90);
                rotateToAngleRate = 0;
                turnController.enable();
                }
                LF.set(rotateToAngleRate * 0.5);
                LB.set(rotateToAngleRate * 0.5);
                RF.set(rotateToAngleRate * 0.5);
                RB.set(rotateToAngleRate * 0.5);
            //}
            */
            if(!turnController.isEnabled())
            {
            target = ahrs.getYaw();
            //turnController.setSetpoint(target);
            rotateToAngleRate = 0;
            turnController.enable();
            }
            
            target = ahrs.getYaw();
            while(timer.get() < 3)
            {
                double leftSpeed; 
                double rightSpeed;
                
                double currentPos = ahrs.getYaw(); 
                
                leftSpeed = 0.5 - (currentPos - target) / 100;
                rightSpeed = (0.5 + (currentPos - target) / 100) * -1;
                /*
                // Left - negative - current pos = -1, target = 0
                leftSpeed = 0.5;
                rightSpeed = -0.5;
                /*
                LF.set((.5 + rotateToAngleRate) * 0.5);
                LB.set((.5 + rotateToAngleRate) * 0.5);
                RF.set((-.5 - rotateToAngleRate) * 0.5);
                RB.set((-.5 - rotateToAngleRate) * 0.5);
                */ 
                
                LB.set(leftSpeed);
                LF.set(leftSpeed);
                RF.set(rightSpeed);
                RB.set(rightSpeed);
            }
            while(timer.get() > 3.2 && timer.get() < 6) {
            	turnController.setSetpoint(kTargetAngleDegrees);
            	
                double leftStickValue = rotateToAngleRate;
                double rightStickValue = rotateToAngleRate;
                
                LF.set(leftStickValue * 0.5);
                LB.set(leftStickValue * 0.5);
                
                RF.set(rightStickValue * 0.5);
                RB.set(rightStickValue * 0.5);
                
                if (Math.abs(ahrs.getAngle() - kTargetAngleDegrees) <= 3 ) {
                LF.set(leftStickValue * 0.1);
                LB.set(leftStickValue * 0.1);
                    
                RF.set(rightStickValue * 0.1);
                RB.set(rightStickValue * 0.1);
                }
                
                else {
                LF.set(leftStickValue * 0.5);
                LB.set(leftStickValue * 0.5);
                    
                RF.set(rightStickValue * 0.5);
                RB.set(rightStickValue * 0.5);
                }
                
                
            	
            }
            System.out.print(ahrs.getAngle());
                System.out.println();
            LF.set(0);
            LB.set(0);
            RF.set(0);
            RB.set(0);
            }
            
            
            //}
            /*else 
            {
                */
                
        //    
        
         
        
    
    

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //simple drivetrain
        
        double left = (player2.getRawAxis(1) * -1); // *-1 to inverse left side | left_stick_y
        double right = player2.getRawAxis(5); // right_stick_y
        
        double intake = (player1.getRawAxis(2) ); // left trigger
        double intake2 = (player1.getRawAxis(3)); // right trigger
        
        
        double basket = (player1.getRawAxis(1) * -1); // left_stick_y
        
        double actBasket;
        if (basket >= 0.5)
        {
            actBasket = 0.5;
        }
        
        else if (basket >= -1)
        {
            actBasket = -1;
        }
    
        LF.set(left);
        LB.set(left);
        RF.set(right);
        RB.set(right);
        
        lift.set(basket);
        
        if (player2.getRawButton(5) && intake >= 0.5) { // left bumper
            midtake();
        }
    
        else {
            stopMotors();
        }
        if (intake >= 0.5)
        {
            intake();
        }
        
        else if (intake2 >= 0.5)
        {
            outtake();
        }
        
        else
        {
            stopMotors();
        }
        /*
        if(player1.getRawButton(2)) // x button
        {
            exSoloIn.set(DoubleSolenoid.Value.kForward);
            exSoloIn2.set(DoubleSolenoid.Value.kForward);
        }
        
        else if (player1.getRawButton(3)) // y button
        {
            exSoloIn.set(DoubleSolenoid.Value.kReverse);
            exSoloIn2.set(DoubleSolenoid.Value.kReverse);
        }
        
        
        if(player1.getRawButton(1)) // b button
        {
            compressor.setClosedLoopControl(true);
        }
        
        else if(player1.getRawButton(4)) // left bumper
        {
            compressor.setClosedLoopControl(false);
        }
        */
        
    }

    /**
     * This function is called periodically during test mode.
     */
    boolean driveToggle = false; // false means Tank Drive, true means Arcade Drive.
    boolean driveLastPass = false;
    @Override
    public void testPeriodic() {
        
        double left = (player2.getRawAxis(1) * -.6); // *-1 to inverse left side | left_stick_y
        double right = (player2.getRawAxis(5) * 0.6); // right_stick_y
        
        double turn = (player2.getRawAxis(1) * -0.75); //  left stick  y
        double power = (player2.getRawAxis(4) * 0.75); // right stick x8
        
        boolean backPressed = (player2.getRawButton(7)); // select/back button
        
        if (backPressed && driveToggle == false) {
            driveToggle = true;
        }

        
        else if (backPressed && driveToggle == true) {
            driveToggle = false;
        }
        
        if (driveToggle == true) {
            LF.set(power + turn);
            LB.set(power + turn);
            
            RF.set(power - turn);
            RB.set(power - turn);    
        }
        
        else {
            LF.set(left);
            LB.set(left);
            
            RF.set(right);
            RB.set(right);    
            driveLastPass = backPressed;
        }
        if ( player2.getRawButton(1)) {
            /* While this button is held down, rotate to target angle.  
             * Since a Tank drive system cannot move forward simultaneously 
             * while rotating, all joystick input is ignored until this
             * button is released.
             */
            if (!turnController.isEnabled()) {
                turnController.setSetpoint(kTargetAngleDegrees);
                rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
                turnController.enable();
            }
            double leftStickValue = rotateToAngleRate;
            double rightStickValue = rotateToAngleRate;
            
            LF.set(leftStickValue * 0.5);
            LB.set(leftStickValue * 0.5);
            
            RF.set(rightStickValue * 0.5);
            RB.set(rightStickValue * 0.5);
            
            if (Math.abs(ahrs.getAngle() - kTargetAngleDegrees) <= 3 ) {
            LF.set(leftStickValue * 0.1);
            LB.set(leftStickValue * 0.1);
                
            RF.set(rightStickValue * 0.1);
        RB.set(rightStickValue * 0.1);
            }
            
            else {
            LF.set(leftStickValue * 0.5);
            LB.set(leftStickValue * 0.5);
                
            RF.set(rightStickValue * 0.5);
            RB.set(rightStickValue * 0.5);
            }
            

        } else if ( player2.getRawButton(3)) {
            /* "Zero" the yaw (whatever direction the sensor is 
             * pointing now will become the new "Zero" degrees.
             */
            ahrs.zeroYaw();
        } else if ( player2.getRawButton(2)) {
            /* While this button is held down, the robot is in
             * "drive straight" mode.  Whatever direction the robot
             * was heading when "drive straight" mode was entered
             * will be maintained.  The average speed of both 
             * joysticks is the magnitude of motion.
             */
            if(!turnController.isEnabled()) {
                // Acquire current yaw angle, using this as the target angle.
                turnController.setSetpoint(ahrs.getYaw());
                rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
                turnController.enable();
            }
            double magnitude = (player2.getRawAxis(1) + player2.getRawAxis(5)) / 2;
            double leftStickValue = magnitude + rotateToAngleRate;
            double rightStickValue = magnitude - rotateToAngleRate;
            LF.set(leftStickValue * -0.5);
            LB.set(leftStickValue * -0.5);
            
            RF.set(rightStickValue * 0.5);
            RB.set(rightStickValue * 0.5);
        } else {
            /* If the turn controller had been enabled, disable it now. */
            if(turnController.isEnabled()) {
                turnController.disable();
            }
            /* Standard tank drive, no driver assistance. */
            
            /*LF.set(left);
            LB.set(left);
            
            RF.set(right);
            RB.set(right);
            */
}
        System.out.print(right);
        System.out.println();
    }
    
    public void intake() {
        iLB.set(.5);
        iLF.set(.5);
        iRB.set(.5);
        iRF.set(-.5);
    }
    
        public void outtake() {
            iLB.set(-.6);
            iLF.set(-.4);
            iRB.set(.6);
            iRF.set(.4);
        }
        
        public void midtake() {
            //iLB.set(.3);
            iLF.set(.5);
            //iRB.set(-.3);
            iRF.set(-.5);
        }
        
        public void mistake() {
                //iLB.set(.3);
                iLF.set(-.5);
                //iRB.set(-.3);
                iRF.set(.5);

        }
        
        public void stopMotors() {
            iLB.set(0);
            iLF.set(0);
            iRB.set(0);
            iRF.set(0);
        }

        @Override
        public void pidWrite(double output) {
             rotateToAngleRate = output;
        }
    
}
