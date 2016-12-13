package org.usfirst.frc.team3238.robot;

/**
 * Reduces unwanted rotation of mecanum chassis by using a gyro sensor and a PI
 * loop
 *
 * @author Nick Papadakis
 */
public class GyroDrive
{
    static double error;
    static double cumulativeError = 0;
    static double adjustedRotationValue;
    static double oldX = 0;
    static double oldY = 0;
    static double time;
    static double oldTime = 0;
    static double timeDifference;
    
    /**
     * Takes the current movement commands for the chassis, PI constants, and
     * current gyroValue
     *
     * @param x
     *            The current x power for the robot's movement
     * @param y
     *            The current y power for the robot's movement
     * @param rotation
     *            The current rotation power for the robot's movement
     * @param pConstant
     *            The P constant for the PI loop
     * @param iConstant
     *            The I constant for the PI loop
     * @param spinThreshold
     *            The gyro value at which the PI loop will stop trying to
     *            correct the robot's spin
     * @param gyroValue
     *            The current value of the gyro sensor
     * @return The adjusted rotation value for correcting the robot's rotation
     */
    static double getAdjustedRotationValue(double x, double y, double rotation,
            double pConstant, double iConstant, double spinThreshold,
            double gyroValue)
    {
        error = Math.abs(gyroValue);
        time = System.currentTimeMillis(); // Reads the system's time
        // If this is the first loop, set timeDifference to 0
        if(oldTime == 0)
        {
            timeDifference = 0;
        }
        else
        {
            timeDifference = time - oldTime;
        }
        // If the joystick input has changed, reset cumulativeError
        if(Math.abs(oldX - x) > 0.01 || Math.abs(oldY - y) > 0.01)
        {
            cumulativeError = 0.0;
        }
        else
        {
            cumulativeError += error;
        }
        /*
         * Check to see if the robot's spinning really fast due to a driver
         * induced spin or if the driver is trying to rotate, and if so, don't
         * change the current rotation value
         */
        if((error > spinThreshold) || rotation <= -0.00001 || rotation >= 0.000001)
        {
            adjustedRotationValue = rotation;
        }
        /*
         * Check the direction of the gyroValue to know which way the chassis
         * needs to rotate in to compensate
         */
        else if(gyroValue > 0)
        {
            adjustedRotationValue = error * pConstant + cumulativeError
                    * iConstant * timeDifference;
        }
        else if(gyroValue < 0)
        {
            adjustedRotationValue = -(error * pConstant + cumulativeError
                    * iConstant * timeDifference);
        }
        else
        {
            adjustedRotationValue = 0;
        }
        // Set up the "old" values for the next loop
        oldX = x;
        oldY = y;
        oldTime = time;
        return adjustedRotationValue;
    }
    
    /**
     * Resets the cumulativeError and oldTime variables
     */
    static void reinit()
    {
        cumulativeError = 0;
        oldTime = 0;
    }
}