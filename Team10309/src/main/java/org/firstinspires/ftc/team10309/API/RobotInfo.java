package org.firstinspires.ftc.team10309.API;

/**
 * API ONLY! Holds constant information about the robot
 */
public abstract class RobotInfo {

    /**
     * The number of pulses in each full revolution of the odometer wheels
     */
    public final int odometerPPR = 360;

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
     * The name of the odometer wheel responsible for measuring the forward and backward movement
     */
    public final static String dOdometerName = "DOdometer";
    /**
     * The name of the odometer wheel responsible for measuring the left and right movement
     */
    public final static String sOdometerName = "SOdometer";
}
