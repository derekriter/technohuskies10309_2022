package org.firstinspires.ftc.team10309.API.info;

/**
 * Holds constant information about the robot
 */
public abstract class RobotInfo {

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
    /**
     * The name of the lift motor
     */
    public final static String liftName = "lift";

    /**
     * The number of ticks per revolution on the drive motors
     */
    public final static int driveTPR = 1120;
    /**
     * The diameter in inches of the drive wheels
     */
    public final static float driveDiameter = 4;
}