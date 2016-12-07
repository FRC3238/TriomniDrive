package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

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
    Joystick mainJoystick;
    
    Chassis chassis;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        try
        {
            navX = new AHRS(SPI.Port.kMXP);
            
            talonOne = new Talon(0);
            talonTwo = new Talon(1);
            talonThree = new Talon(2);
            
            mainJoystick = new Joystick(0);
            
            chassis = new Chassis(talonOne, talonTwo, talonThree, mainJoystick, navX);
        } catch(Exception e)
        {
            DriverStation.reportError(e.getMessage(), true);
        }
    }
    
    /**
     * This method is called periodically during disabled
     */
    public void disabledPeriodic()
    {
        chassis.setEnabled(false);
    }
    
    /**
     * This method is called once at the beginning of autonomous
     */
    public void autonomousInit()
    {
        
    }
    
    /**
     * This method is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
        
    }
    
    /**
     * This method is called once at the beginning of operator control
     */
    public void teleopInit()
    {
        chassis.init();
    }
    
    /**
     * This method is called periodically during operator control
     */
    public void teleopPeriodic()
    {
        chassis.run();
    }
    
    /**
     * This method is called periodically during test mode
     */
    public void testPeriodic()
    {
        
    }
    
}