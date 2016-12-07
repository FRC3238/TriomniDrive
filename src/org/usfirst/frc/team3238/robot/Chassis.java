package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

/**
 * Created by aaron on 11/4/2016.
 */
public class Chassis implements PIDSource, PIDOutput
{
    private AHRS navX;
    
    private Talon[] talons = new Talon[4];
    private Joystick joy;
    private PIDController pid;
    
    private boolean enabled;
    private double pidOutput;
    private double pidInput;
    private PIDSourceType pidSourceType;
    
    private static final double DEADZONE = 0.1;
    private static final double TWIST_THRESH = 0.3;
    private static final String pDash = "DB/Slider 0";
    private static final String iDash = "DB/Slider 1";
    private static final String dDash = "DB/Slider 2";
    private double pConst = 1.0;
    private double iConst = 0.0;
    private double dConst = 0.0;
    
    /**
     * Constructor to pass talons to chassis class, set joystick to be used, and
     * initially disable the chassis.
     *
     * @param talonOne   First talon
     * @param talonTwo   Second talon, clockwise of first
     * @param talonThree Third talon, clockwise of first and second
     * @param joystick   Joystick object to control chassis
     */
    Chassis(Talon talonOne, Talon talonTwo, Talon talonThree, Joystick joystick,
            AHRS navX)
    {
        this.navX = navX;
        
        talons[1] = talonOne;
        talons[2] = talonTwo;
        talons[3] = talonThree;
        setEnabled(false);
        
        pid = new PIDController(pConst, iConst, dConst, this, this, 15);
        pid.setInputRange(-2000.0, 2000.0);
        pid.setOutputRange(-1.0, 1.0);
        pid.setSetpoint(0.0);
        pid.setAbsoluteTolerance(5);
        
        if(SmartDashboard.getNumber(pDash) == 0.0)
        {
            SmartDashboard.putNumber(pDash, 0.1);
        }
        
        joy = joystick;
    }
    
    /**
     * Initializes chassis, to be called in init for each state to be ran in,
     * not in robotInit(). Currently calls enable().
     */
    public void init()
    {
        setEnabled(true);
        pid.setPID(SmartDashboard.getNumber(pDash, 0.1), SmartDashboard.getNumber(iDash, 0.0), SmartDashboard.getNumber(dDash, 0.0));
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
    
    public void run()
    {
        double[] speeds = { 0.0, 0.0, 0.0, 0.0 };
        boolean changeEnable = false;
    
        SmartDashboard.putNumber("Robot Direction", navX.getAngle());
        SmartDashboard.putNumber("Joystick Direction", joy.getDirectionDegrees());
    
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
        } else
        {
            changeEnable = false;
        }
    
        if(enabled)
        {
            double x = getCartesianX(joy.getDirectionDegrees(), joy.getMagnitude());
            double y = getCartesianY(joy.getDirectionDegrees(), joy.getMagnitude());
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
     * Runs chassis off of values from the joystick passed through the
     * constructor.
     */
    public void pidRun()
    {
        double[] speeds = { 0.0, 0.0, 0.0, 0.0 };
        boolean changeEnable = false;
        
        SmartDashboard.putNumber("Robot Direction", navX.getAngle());
        SmartDashboard.putNumber("Joystick Direction", joy.getDirectionDegrees());
        
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
        } else
        {
            changeEnable = false;
        }
        
        if(enabled)
        {
            pid.enable();
            double x = getCartesianX(joy.getDirectionDegrees(), joy.getMagnitude());
            double y = getCartesianY(joy.getDirectionDegrees(), joy.getMagnitude());
            double twist;
            if(Math.abs(joy.getTwist()) >= TWIST_THRESH)
            {
                twist = joy.getTwist();
                pidInput = 0.0;
            } else
            {
                pidInput = navX.getRawGyroZ();
                twist = pidOutput;
            }
    
            dataDump(x, y, twist);
            
            speeds[1] = getSpeedOne(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedThree(x, y, twist);
    
            talons[1].set(speeds[1]);
            talons[2].set(speeds[2]);
            talons[3].set(speeds[3]);
        } else
        {
            pid.disable();
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
     * @param x
     * @param y
     * @param twist
     * @return speed of motor one, 0 - 1
     */
    private double getSpeedOne(double x, double y, double twist)
    {
        //        return ((-1 / 2) * getAboveDeadzone(x)) - ((Math.sqrt(3) / 2) * getAboveDeadzone(y)) + getAboveDeadzone(twist);
        return (-getAboveDeadzone(x) / 2) - ((Math.sqrt(3) / 2) * getAboveDeadzone(y)) + getAboveDeadzone(twist / 1.6, true);
    }
    
    /**
     * Gets speed of motor two. Affects both auto and teleop run methods.
     *
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
     *
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
    
    public void dataDump(double x, double y, double twist)
    {
        SmartDashboard.putString("DB/String 0", "Raw Twist:   " + navX.getRawGyroZ());
        SmartDashboard.putString("DB/String 1", "Twist:       " + joy.getTwist());
        SmartDashboard.putString("DB/String 2", "PID output:  " + pid.get());
        SmartDashboard.putString("DB/String 3", "PID Enabled: " + pid.isEnabled());
        SmartDashboard.putString("DB/String 4", "PID Intput:  " + pidInput);
        SmartDashboard.putString("DB/String 5", "PID Setpoint: " + pid.getSetpoint());
        SmartDashboard.putNumber("Speed one", getSpeedOne(x, y, twist));
        SmartDashboard.putNumber("Speed two", getSpeedTwo(x, y, twist));
        SmartDashboard.putNumber("Speed three", getSpeedThree(x, y, twist));
    }
    
    @Override public void pidWrite(double output)
    {
        pidOutput = output;
    }
    
    @Override public void setPIDSourceType(PIDSourceType pidSource)
    {
        pidSourceType = pidSource;
    }
    
    @Override public PIDSourceType getPIDSourceType()
    {
        return pidSourceType;
    }
    
    @Override public double pidGet()
    {
        return pidInput;
    }
}
