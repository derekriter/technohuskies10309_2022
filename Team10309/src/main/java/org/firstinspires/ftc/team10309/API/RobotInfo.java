package org.firstinspires.ftc.team10309.API;

/**
 * API ONLY! Holds constant information about the robot
 */
public abstract class RobotInfo {

    /**
     * The number of ticks in full rotation of the odometer wheels
     */
    public final static int odometerTicks = 0;
    /**
     * The circumference of the odometer wheels
     */
    public final static float odometerCirc = 0;
    /**
     * The distance of the left and right odometer wheels from the center of the robot (on the x
     * axis)
     */
    public final static float odometerDist = 0;
    /**
     * The circumference of the circle from the left to the right odometer wheels
     */
    public final static float robotCirc = (float) (2 * Math.PI) * odometerDist;

    /**
     * The name of the front left wheel
     */
    public final static String flMotorName = "front left";
    /**
     * The name of the front right wheel
     */
    public final static String frMotorName = "front right";
    /**
     * The name of the back left wheel
     */
    public final static String blMotorName = "back left";
    /**
     * The name of the back right wheel
     */
    public final static String brMotorName = "back right";
}
