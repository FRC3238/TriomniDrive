package org.usfirst.frc.team3238.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author aaron
 * @version 2.0 <p>Purpose: to power a three-wheeled kiwi drive chassis.</p>
 */
public class Chassis
{
    private AHRS navX;
    
    private Talon[] talons = new Talon[4];
    private Encoder[] encoders = new Encoder[4];
    private Joystick joyOne;
    private Joystick joyTwo;
    private boolean enabled;
    boolean changeEnable = false;
    
    private static final double DEADZONE = 0.1;
    private static final double TWIST_THRESH = 0.5;
    private static final double P_CONST = -0.6;
    private static final double I_CONST = 0.000001;
    private static final double WHEEL_ONE_X_CORD = 0.0;
    private static final double WHEEL_ONE_Y_CORD = 1.0;
    private static final double WHEEL_TWO_X_CORD = Math.sqrt(3.0) / 2;
    private static final double WHEEL_TWO_Y_CORD = -1 / 2;
    private static final double WHEEL_THREE_X_CORD = -Math.sqrt(3.0) / 2;
    private static final double WHEEL_THREE_Y_CORD = -1 / 2;
    
    /**
     * Constructor to pass talons to chassis class, set joystickOne to be used,
     * and initially disable the chassis.
     *
     * @param talonOne    First talon
     * @param talonTwo    Second talon, clockwise of first
     * @param talonThree  Third talon, clockwise of first and second
     * @param joystickOne Joystick object to control chassis
     */
    Chassis(Talon talonOne, Talon talonTwo, Talon talonThree,
            Encoder encoderOne, Encoder encoderTwo, Encoder encoderThree,
            Joystick joystickOne, Joystick joystickTwo, AHRS navX)
    
    {
        this.navX = navX;
        
        talons[1] = talonOne;
        talons[2] = talonTwo;
        talons[3] = talonThree;
        encoders[1] = encoderOne;
        encoders[2] = encoderTwo;
        encoders[3] = encoderThree;
        setEnabled(false);
        
        joyOne = joystickOne;
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
        
        if(joyOne.getRawButton(2))
        {
            navX.reset();
        }
        
        if(joyOne.getRawButton(11) && enabled && !changeEnable)
        {
            setEnabled(false);
            changeEnable = true;
        } else if(joyOne.getRawButton(11) && !enabled && !changeEnable)
        {
            setEnabled(true);
            changeEnable = true;
        } else if(changeEnable)
        {
            changeEnable = false;
        }
        
        if(enabled)
        {
            double x = getCartesianX(joyOne.getDirectionDegrees(),
                    joyOne.getMagnitude());
            double y = getCartesianY(joyOne.getDirectionDegrees(),
                    joyOne.getMagnitude());
            double twist;
            if(Math.abs(joyOne.getTwist()) > TWIST_THRESH)
            {
                twist = joyOne.getTwist();
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
        
        if(joyOne.getRawButton(2))
        {
            navX.reset();
        }
        if(joyOne.getRawButton(1))
        {
            resetEncoders();
        }
        
        if(joyOne.getRawButton(11) && enabled && !changeEnable)
        {
            setEnabled(false);
            changeEnable = true;
        } else if(joyOne.getRawButton(11) && !enabled && !changeEnable)
        {
            setEnabled(true);
            changeEnable = true;
        } else if(changeEnable)
        {
            changeEnable = false;
        }
        
        if(enabled)
        {
            double x = getCartesianX(joyOne.getDirectionDegrees(),
                    joyOne.getMagnitude());
            double y = getCartesianY(joyOne.getDirectionDegrees(),
                    joyOne.getMagnitude());
            double twist;
            
            if(getAboveTwistDeadzone(joyOne.getTwist()))
            {
                
                twist = GyroDrive.getAdjustedRotationValue(x, y,
                        (joyOne.getTwist() * joyOne.getTwist()), P_CONST,
                        I_CONST, 0.25, navX.getRate());
            } else
            {
                twist = joyOne.getTwist();
            }
            
            speeds[1] = getSpeedOne(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedThree(x, y, twist);
            
            double highVal = 1.0;
            for(int i = 1; i < speeds.length; i++)
            {
                if(speeds[i] > highVal)
                {
                    highVal = speeds[i];
                }
            }
            
            for(int i = 1; i < speeds.length; i++)
            {
                speeds[i] /= highVal;
            }
            
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
                * getAboveDeadzone(y)) + getAdjustedTwist(twist,
                WHEEL_ONE_X_CORD, WHEEL_ONE_Y_CORD);
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
                * getAboveDeadzone(y)) + getAdjustedTwist(twist,
                WHEEL_TWO_X_CORD, WHEEL_TWO_Y_CORD);
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
        return getAboveDeadzone(x) + getAdjustedTwist(twist, WHEEL_THREE_X_CORD,
                WHEEL_THREE_Y_CORD);
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
    private boolean getAboveTwistDeadzone(double val)
    {
        if(Math.abs(val) > TWIST_THRESH)
        {
            return true;
        } else
        {
            return false;
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
     * Gets new value of twist for each wheel given the center of rotation and
     * coordinates of each wheel.
     *
     * @param twist  rotation speed
     * @param wheelX x coordinate of wheel
     * @param wheelY y coordinate of wheel
     * @return adjusted twist speed
     */
    private double getAdjustedTwist(double twist, double wheelX, double wheelY)
    {
        double adjustedTwist;
        double magnitude = 2 * joyTwo.getMagnitude();
        double direction = 2 * joyTwo.getDirectionDegrees();
        double x = getCartesianX(direction, magnitude);
        double y = getCartesianY(direction, magnitude);
        adjustedTwist = Math
                .sqrt(((wheelX - x) * (wheelX - x)) + ((wheelY - y) * (wheelY
                        - y)));
        adjustedTwist *= twist;
        return adjustedTwist;
    }
    
    /**
     * Gets experimental x joystick value based on encoder wheel speeds
     *
     * @param sOne   speed of wheel one
     * @param sTwo   speed of wheel two
     * @param sThree speed of wheel three
     * @return experimental x joystick value
     */
    public double getEncX(double sOne, double sTwo, double sThree)
    {
        return ((sOne + sTwo - (2 * sThree)) * Math.sqrt(3.0)) / (sOne - sTwo
                - (2 * Math.sqrt(3.0)));
    }
    
    /**
     * Gets experimental y joystick value based on encoder wheel speeds
     *
     * @param sOne   speed of wheel one
     * @param sThree speed of wheel three
     * @param encX   experimental x joystick value
     * @return experimental y joystick value
     */
    public double getEncY(double sOne, double sThree, double encX)
    {
        return (-2 * (sOne - sThree + encX)) / (encX + Math.sqrt(3.0));
    }
    
    /**
     * Gets experimental twist joystick value based on e
     *
     * @param sThree speed of wheel three
     * @param x      experimental x joystick value
     * @return experimental twist joystick value
     */
    public double getEncTwist(double sThree, double x)
    {
        return sThree - x;
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
        double encOne = encoders[1].getRate();
        double encTwo = encoders[2].getRate();
        double encThree = encoders[3].getRate();
        double encX = getEncX(encOne, encTwo, encThree);
        double encY = getEncY(encOne, encThree, encX);
        double encTwist = getEncTwist(encThree, encX);
        SmartDashboard.putString("DB/String 0", "X:     " + joyOne.getX());
        SmartDashboard.putString("DB/String 1", "Y:     " + joyOne.getY());
        SmartDashboard.putString("DB/String 2", "Twist: " + joyOne.getTwist());
        SmartDashboard.putString("DB/String 5", "EX:     " + encX);
        SmartDashboard.putString("DB/String 6", "EY:     " + encY);
        SmartDashboard.putString("DB/String 7", "ETwist: " + encTwist);
        SmartDashboard.putNumber("Speed one", getSpeedOne(x, y, twist));
        SmartDashboard.putNumber("Speed two", getSpeedTwo(x, y, twist));
        SmartDashboard.putNumber("Speed three", getSpeedThree(x, y, twist));
        SmartDashboard.putNumber("Robot Direction", navX.getAngle());
        SmartDashboard
                .putNumber("Joystick Direction", joyOne.getDirectionDegrees());
    }
}
