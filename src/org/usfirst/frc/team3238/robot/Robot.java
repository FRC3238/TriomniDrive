package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
    AHRS navX;
    
    Talon talonOne, talonTwo, talonThree;
    Encoder encoderOne, encoderTwo, encoderThree;
    Joystick mainJoystick;
    Joystick assistJoystick;
    Chassis chassis;
    Preferences prefs;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override public void robotInit()
    {
        try
        {
            prefs = Preferences.getInstance();
            navX = new AHRS(SPI.Port.kMXP);
            
            talonOne = new Talon(0);
            talonTwo = new Talon(1);
            talonThree = new Talon(2);
            
            encoderOne = new Encoder(0, 1);
            encoderTwo = new Encoder(2, 3);
            encoderThree = new Encoder(4, 5);
            
            mainJoystick = new Joystick(0);
            assistJoystick = new Joystick(1);
            chassis = new Chassis(talonOne, talonTwo, talonThree, encoderOne,
                    encoderTwo, encoderThree, mainJoystick, assistJoystick,
                    navX);
            new Thread(new VisionProc()).start();
        } catch(Exception e)
        {
            DriverStation.reportError(e.getMessage(), true);
        }
    }
    
    /**
     * This method is called periodically during disabled
     */
    @Override public void disabledPeriodic()
    {
        
    }
    
    /**
     * This method is called once at the beginning of autonomous
     */
    @Override public void autonomousInit()
    {
        
    }
    
    /**
     * This method is called periodically during autonomous
     */
    @Override public void autonomousPeriodic()
    {
        
    }
    
    /**
     * This method is called once at the beginning of operator control
     */
    @Override public void teleopInit()
    {
        chassis.init(prefs.getDouble("Chassis deadzone", chassis.deadzone),
                prefs.getDouble("Twist deadzone", chassis.twistThresh),
                prefs.getDouble("Gyro drive P", chassis.gyroP),
                prefs.getDouble("Gyro drive I", chassis.gyroI));
    }
    
    /**
     * This method is called periodically during operator control
     */
    @Override public void teleopPeriodic()
    {
        chassis.pidRun();
    }
    
    /**
     * This method is called once at the beginning of test mode
     */
    @Override public void testInit()
    {
        chassis.init(prefs.getDouble("Chassis deadzone", chassis.deadzone),
                prefs.getDouble("Twist deadzone", chassis.twistThresh),
                prefs.getDouble("Gyro drive P", chassis.gyroP),
                prefs.getDouble("Gyro drive I", chassis.gyroI));
    }
    
    /**
     * This method is called periodically during test mode
     */
    @Override public void testPeriodic()
    {
        chassis.run();
    }
}