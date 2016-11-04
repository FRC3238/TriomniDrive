package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

/**
 * Created by aaron on 11/4/2016.
 */
public class Chassis
{
    Talon[] talons;
    Joystick joy;
    boolean enabled;

    Chassis(Talon talonOne, Talon talonTwo, Talon talonThree, Joystick joystick)
    {
        talons[1] = talonOne;
        talons[2] = talonTwo;
        talons[3] = talonThree;
        disable();

        joy = joystick;
    }

    public void init()
    {
        enable();
    }

    public void enable()
    {
        enabled = true;
    }

    public void disable()
    {
        enabled = false;
    }

    public void run()
    {
        double[] speeds = {0.0, 0.0, 0.0, 0.0};

        if(enabled)
        {
            double x = joy.getX();
            double y = joy.getY();
            double twist = joy.getTwist();

            speeds[1] = ((-1/2) * x) - ((Math.sqrt(3)/2) * y) + twist;
            speeds[2] = ((-1/2) * x) + ((Math.sqrt(3)/2) * y) + twist;
            speeds[3] = x + twist;

            talons[1].set(speeds[1]);
            talons[2].set(speeds[2]);
            talons[3].set(speeds[3]);
        }
    }

    public void autoRun(double x, double y, double twist)
    {
        if(enabled)
        {
            double[] speeds = {0.0, 0.0, 0.0, 0.0};

            if(enabled)
            {
                speeds[1] = ((-1/2) * x) - ((Math.sqrt(3)/2) * y) + twist;
                speeds[2] = ((-1/2) * x) + ((Math.sqrt(3)/2) * y) + twist;
                speeds[3] = x + twist;

                talons[1].set(speeds[1]);
                talons[2].set(speeds[2]);
                talons[3].set(speeds[3]);
            }
        }
    }
}
