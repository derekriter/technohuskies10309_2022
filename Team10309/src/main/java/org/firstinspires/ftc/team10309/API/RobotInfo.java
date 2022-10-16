package org.firstinspires.ftc.team10309.API;

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
     * The name of channel A of the odometer wheel responsible for measuring the forward and
     * backward movement
     */
    public final static String dOdometerAName = "DOdometerA";
    /**
     * The name of channel B of the odometer wheel responsible for measuring the forward and
     * backward movement
     */
    public final static String dOdometerBName = "DOdometerB";
    /**
     * The name of channel A of the odometer wheel responsible for measuring the left and
     * right movement
     */
    public final static String sOdometerAName = "SOdometerA";
    /**
     * The name of channel B of the odometer wheel responsible for measuring the left and
     * right movement
     */
    public final static String sOdometerBName = "SOdometerB";
}
