package org.usfirst.frc.team3238.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author aaron
 * @version 2.0
 *          <p>
 *          Purpose: to power a three-wheeled kiwi drive chassis.
 */
public class Chassis
{
    private AHRS navX;
    
    private Talon[] talons = new Talon[4];
    private Encoder[] encoders = new Encoder[4];
    private Joystick joy;
    
    private boolean enabled;
    boolean changeEnable = false;
    
    private static final double DEADZONE = 0.1;
    private static final double TWIST_THRESH = 0.5;
    private final double P_CONST = -0.6;
    private final double I_CONST = 0.000001;
    
    /**
     * Constructor to pass talons to chassis class, set joystick to be used, and
     * initially disable the chassis.
     *
     * @param talonOne   First talon
     * @param talonTwo   Second talon, clockwise of first
     * @param talonThree Third talon, clockwise of first and second
     * @param joystick   Joystick object to control chassis
     */
    Chassis(Talon talonOne, Talon talonTwo, Talon talonThree,
            Encoder encoderOne, Encoder encoderTwo, Encoder encoderThree,
            Joystick joystick, AHRS navX)
    {
        this.navX = navX;
        
        talons[1] = talonOne;
        talons[2] = talonTwo;
        talons[3] = talonThree;
        encoders[1] = encoderOne;
        encoders[2] = encoderTwo;
        encoders[3] = encoderThree;
        setEnabled(false);
        
        joy = joystick;
    }
    
    /**
     * Initializes chassis, to be called in init for each state to be ran in,
     * not in robotInit(). Currently calls enable().
     */
    public void init()
    {
        GyroDrive.reinit();
        setEnabled(true);
        navX.reset();
        resetEncoders();
    }
    
    private void resetEncoders()
    {
        for(int i = 1; i < encoders.length; i++)
        {
            encoders[i].reset();
        }
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
        
        if(joy.getRawButton(2))
        {
            navX.reset();
        }
        
        if(joy.getRawButton(11) && enabled && !changeEnable)
        {
            setEnabled(false);
            changeEnable = true;
        } else if(joy.getRawButton(11) && !enabled && !changeEnable)
        {
            setEnabled(true);
            changeEnable = true;
        } else if(changeEnable)
        {
            changeEnable = false;
        }
        
        if(enabled)
        {
            double x = getCartesianX(joy.getDirectionDegrees(),
                    joy.getMagnitude());
            double y = getCartesianY(joy.getDirectionDegrees(),
                    joy.getMagnitude());
            double twist;
            if(Math.abs(joy.getTwist()) > TWIST_THRESH)
            {
                twist = joy.getTwist();
            } else
            {
                twist = 0.0;
            }
            
            speeds[1] = getSpeedOne(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedThree(x, y, twist);
            
            SmartDashboard.putNumber("Speed one", getSpeedOne(x, y, twist));
            SmartDashboard.putNumber("Speed two", getSpeedTwo(x, y, twist));
            SmartDashboard.putNumber("Speed three", getSpeedThree(x, y, twist));
            
            dataDump(x, y, twist);
            
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
     * Runs the chassis with pid controlled gyro drive.
     */
    public void pidRun()
    {
        double[] speeds = { 0.0, 0.0, 0.0, 0.0 };
        
        if(joy.getRawButton(2))
        {
            navX.reset();
        }
        if(joy.getRawButton(1))
        {
            resetEncoders();
        }
        
        if(joy.getRawButton(11) && enabled && !changeEnable)
        {
            setEnabled(false);
            changeEnable = true;
        } else if(joy.getRawButton(11) && !enabled && !changeEnable)
        {
            setEnabled(true);
            changeEnable = true;
        } else if(changeEnable)
        {
            changeEnable = false;
        }
        
        if(enabled)
        {
            double x = getCartesianX(joy.getDirectionDegrees(),
                    joy.getMagnitude());
            double y = getCartesianY(joy.getDirectionDegrees(),
                    joy.getMagnitude());
            double twist = GyroDrive.getAdjustedRotationValue(x, y,
                    getAboveTwistDeadzone(joy.getTwist()), P_CONST, I_CONST,
                    0.25, navX.getRate());
            
            speeds[1] = getSpeedOne(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedThree(x, y, twist);
            
            dataDump(x, y, twist);
            
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
     *
     * @param x     joystick x value
     * @param y     joystick y value
     * @param twist joystick twist value
     * @return speed of motor one, 0 - 1
     */
    private double getSpeedOne(double x, double y, double twist)
    {
        //        return ((-1 / 2) * getAboveTwistDeadzone(x)) - ((Math.sqrt(3) / 2) * getAboveTwistDeadzone(y)) + getAboveTwistDeadzone(twist);
        return (-getAboveDeadzone(x) / 2) - ((Math.sqrt(3) / 2)
                * getAboveDeadzone(y)) + twist;
    }
    
    /**
     * Gets speed of motor two. Affects both auto and teleop run methods.
     *
     * @param x     joystick x value
     * @param y     joystick y value
     * @param twist joystick twist value
     * @return speed of motor two, 0 - 1
     */
    private double getSpeedTwo(double x, double y, double twist)
    {
        //        return ((-1 / 2) * getAboveTwistDeadzone(x)) + ((Math.sqrt(3) / 2) * getAboveTwistDeadzone(y)) + getAboveTwistDeadzone(twist);
        return (-getAboveDeadzone(x) / 2) + ((Math.sqrt(3) / 2)
                * getAboveDeadzone(y)) + twist;
    }
    
    /**
     * Gets speed of motor three. Affects both auto and teleop run methods.
     *
     * @param x     joystick x value
     * @param y     joystick y value
     * @param twist joystick twist value
     * @return speed of motor three, 0 - 1
     */
    private double getSpeedThree(double x, double y, double twist)
    {
        //        return getAboveTwistDeadzone(x) + getAboveTwistDeadzone(twist);
        return getAboveDeadzone(x) + twist;
    }
    
    /**
     * Determines whether the value is above or below the deadzone, and
     * compensates for the deadzone.
     *
     * @param val joystick value
     * @return val if above deadzone, 0 if not
     */
    private double getAboveDeadzone(double val)
    {
        if(Math.sqrt(val * val) > DEADZONE)
        {
            return val;
        } else
        {
            return 0.0;
        }
    }
    
    /**
     * Determines whether the value is above or below the twist deadzone, and
     * compensates for the daedzone.
     *
     * @param val joystick value
     * @return val if above twist deadzone, 0 if not
     */
    private double getAboveTwistDeadzone(double val)
    {
        if(val > TWIST_THRESH)
        {
            return val * val;
        } else if(val < TWIST_THRESH)
        {
            return -val * val;
        } else
        {
            return 0.0;
        }
    }
    
    /**
     * Converts polar coordinates into cartesian coordinates, using chassis
     * angle to implement headless mode.
     *
     * @param direction direction of joystick, in degrees
     * @param magnitude magnitude of joystick
     * @return x value of joystick, compensated for chassis turning
     */
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
    
    /**
     * Converts polar coordinates into cartesian coordinates, using chassis
     * angle to implement headless mode.
     *
     * @param direction direction of joystick, in degrees
     * @param magnitude magnitude of joystick
     * @return y value of joystick, compensated for chassis turning
     */
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
    
    /**
     * Sends relevant data to the smart dashboard
     *
     * @param x     x speed value
     * @param y     y speed value
     * @param twist twist speed value
     */
    public void dataDump(double x, double y, double twist)
    {
        SmartDashboard
                .putString("DB/String 0", "Raw Twist:    " + navX.getRate());
        SmartDashboard
                .putString("DB/String 1", "Twist:        " + joy.getTwist());
        SmartDashboard.putString("DB/String 2", "Twist Adjust: " + twist);
        SmartDashboard.putString("DB/String 5", "Encoder 1: " + encoders[1].get() + " " + encoders[1].getRate());
        SmartDashboard.putString("DB/String 6", "Encoder 2: " + encoders[2].get() + " " + encoders[2].getRate());
        SmartDashboard.putString("DB/String 7", "Encoder 3: " + encoders[3].get() + " " + encoders[3].getRate());
        SmartDashboard.putNumber("Speed one", getSpeedOne(x, y, twist));
        SmartDashboard.putNumber("Speed two", getSpeedTwo(x, y, twist));
        SmartDashboard.putNumber("Speed three", getSpeedThree(x, y, twist));
        SmartDashboard.putNumber("Robot Direction", navX.getAngle());
        SmartDashboard
                .putNumber("Joystick Direction", joy.getDirectionDegrees());
    }
}
