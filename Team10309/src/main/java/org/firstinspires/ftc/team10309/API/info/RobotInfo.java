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

    public final static String imuName = "imu";

    /**
     * The number of ticks per revolution on the drive motors
     */
    public final static int protoDriveTPR = 1120;
    /**
     * The diameter in inches of the drive wheels
     */
    public final static float protoDriveDiameter = 4;
    /**
     * The number of ticks per revolution on the drive motors
     */
    public final static int finalDriveTPR = 560;
    /**
     * The diameter in inches of the drive wheels
     */
    public final static float finalDriveDiameter = 4;
}