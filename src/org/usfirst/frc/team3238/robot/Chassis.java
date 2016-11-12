package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

/**
 * Created by aaron on 11/4/2016.
 */
public class Chassis
{
    private Talon[] talons = new Talon[4];
    private Joystick joy;
    
    private boolean enabled;
    
    /**
     * Constructor to pass talons to chassis class, set joystick to be used, and
     * initially disable the chassis.
     *
     * @param talonOne   First talon
     * @param talonTwo   Second talon, clockwise of first
     * @param talonThree Third talon, clockwise of first and second
     * @param joystick   Joystick object to control chassis
     */
    Chassis(Talon talonOne, Talon talonTwo, Talon talonThree, Joystick joystick)
    {
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
        
        if(enabled)
        {
            double x = joy.getX();
            double y = joy.getY();
            double twist = joy.getTwist();
            
            speeds[1] = getSpeedOne(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedThree(x, y, twist);
            
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
            
            speeds[1] = getSpeedOne(x, y, twist);
            speeds[2] = getSpeedTwo(x, y, twist);
            speeds[3] = getSpeedThree(x, y, twist);
            
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
        return ((-1 / 2) * x) - ((Math.sqrt(3) / 2) * y) + twist;
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
        return ((-1 / 2) * x) + ((Math.sqrt(3) / 2) * y) + twist;
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
        return x + twist;
    }
}
