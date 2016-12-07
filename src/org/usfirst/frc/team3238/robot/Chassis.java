package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

/**
 * Created by aaron on 11/4/2016.
 */
public class Chassis
{
    private AHRS navX;
    
    private Talon[] talons = new Talon[4];
    private Joystick joy;
    
    private boolean enabled;
    private static final double DEADZONE = 0.1;
    
    /**
     * Constructor to pass talons to chassis class, set joystick to be used, and
     * initially disable the chassis.
     *
     * @param talonOne   First talon
     * @param talonTwo   Second talon, clockwise of first
     * @param talonThree Third talon, clockwise of first and second
     * @param joystick   Joystick object to control chassis
     */
    Chassis(Talon talonOne, Talon talonTwo, Talon talonThree, Joystick joystick, AHRS navX)
    {
        this.navX = navX;
        
        talons[1] = talonOne;
        talons[2] = talonTwo;
        talons[3] = talonThree;
        setEnabled(false);
        
        joy = joystick;
    }
    
    /**
     * Initializes chassis, to be called in init for each state to be ran in,
     * not in robotInit(). Currently calls enable().
     */
    public void init()
    {
        setEnabled(true);
        navX.reset();
    }
    
    /**
     * Allows chassis to run if true, stops all motors if false.
     */
    public void setEnabled(boolean enabled)
    {
        this.enabled = enabled;
    }
    
    /**
     * Gets whether chassis is enabled or disabled.
     *
     * @return true for enabled, false for disabled.
     */
    public boolean isEnabled()
    {
        return enabled;
    }
    
    /**
     * Runs chassis off of values from the joystick passed through the
     * constructor.
     */
    public void run()
    {
        double[] speeds = { 0.0, 0.0, 0.0, 0.0 };
        
        SmartDashboard.putNumber("Robot Direction", navX.getAngle());
        SmartDashboard.putNumber("Joystick Direction", joy.getDirectionDegrees());
        
        if(joy.getRawButton(2))
        {
            navX.reset();
        }
        
        if(joy.getRawButton(11) && enabled)
        {
            setEnabled(false);
        } else if(joy.getRawButton(11) && !enabled)
        {
            setEnabled(true);
        }
        
        if(enabled)
        {
            double x = getCartesianX(joy.getDirectionDegrees(), joy.getMagnitude());
            double y = getCartesianY(joy.getDirectionDegrees(), joy.getMagnitude());
            double twist = joy.getTwist() * 0.75;
            
            speeds[1] = getSpeedOne(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedThree(x, y, twist);
            
            SmartDashboard.putNumber("Speed one", getSpeedOne(x, y, twist));
            SmartDashboard.putNumber("Speed two", getSpeedTwo(x, y, twist));
            SmartDashboard.putNumber("Speed three", getSpeedThree(x, y, twist));

            talons[1].set(speeds[1]);
            talons[2].set(speeds[2]);
            talons[3].set(speeds[3]);
        } else
        {
            talons[1].set(0.0);
            talons[2].set(0.0);
            talons[3].set(0.0);
        }
    }
    
    /**
     * Runs chassis off of values passed through this method. To be used for
     * testing or autonomous. All values should be between 0 and 1.
     *
     * @param x     lateral movement
     * @param y     forward/backward movement
     * @param twist twist movement
     */
    public void autoRun(double x, double y, double twist)
    {
        if(enabled)
        {
            double[] speeds = { 0.0, 0.0, 0.0, 0.0 };
            
            speeds[1] = getSpeedThree(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedOne(x, y, twist);
            
            talons[1].set(speeds[1]);
            talons[2].set(speeds[2]);
            talons[3].set(speeds[3]);
        } else
        {
            talons[1].set(0.0);
            talons[2].set(0.0);
            talons[3].set(0.0);
        }
    }
    
    /**
     * Gets speed of motor one. Affects both auto and teleop run methods.
     * @param x
     * @param y
     * @param twist
     * @return speed of motor one, 0 - 1
     */
    private double getSpeedOne(double x, double y, double twist)
    {
//        return ((-1 / 2) * getAboveDeadzone(x)) - ((Math.sqrt(3) / 2) * getAboveDeadzone(y)) + getAboveDeadzone(twist);
        return (-getAboveDeadzone(x) / 2 ) - ((Math.sqrt(3) / 2) * getAboveDeadzone(y)) + getAboveDeadzone(twist / 1.6, true);
    }
    
    /**
     * Gets speed of motor two. Affects both auto and teleop run methods.
     * @param x
     * @param y
     * @param twist
     * @return speed of motor two, 0 - 1
     */
    private double getSpeedTwo(double x, double y, double twist)
    {
//        return ((-1 / 2) * getAboveDeadzone(x)) + ((Math.sqrt(3) / 2) * getAboveDeadzone(y)) + getAboveDeadzone(twist);
        return (-getAboveDeadzone(x) / 2) + ((Math.sqrt(3) / 2) * getAboveDeadzone(y)) + getAboveDeadzone(twist / 1.6, true);
    }
    
    /**
     * Gets speed of motor three. Affects both auto and teleop run methods.
     * @param x
     * @param y
     * @param twist
     * @return speed of motor three, 0 - 1
     */
    private double getSpeedThree(double x, double y, double twist)
    {
//        return getAboveDeadzone(x) + getAboveDeadzone(twist);
        return getAboveDeadzone(x) + getAboveDeadzone(twist / 1.6, true);
    }
    
    private double getAboveDeadzone(double val)
    {
        if(Math.abs(val) >= DEADZONE)
        {
            return val;
        } else
        {
            return 0.0;
        }
    }
    
    private double getAboveDeadzone(double val, boolean twist)
    {
        if(Math.abs(val) >= 2 * DEADZONE)
        {
            return val;
        } else
        {
            return 0.0;
        }
    }
    
    private double getCartesianX(double direction, double magnitude)
    {
        double degrees = navX.getAngle();
        direction -= degrees;
        while(direction > 360)
        {
            direction -= 360;
        }
        while(direction < 0)
        {
            direction += 360;
        }
        
        return (magnitude * Math.cos(Math.toRadians(direction)));
    }
    private double getCartesianY(double direction, double magnitude)
    {
        double degrees = navX.getAngle();
        direction -= degrees;
        while(direction > 360)
        {
            direction -= 360;
        }
        while(direction < 0)
        {
            direction += 360;
        }
        
        return (magnitude * Math.sin(Math.toRadians(direction)));
    }
