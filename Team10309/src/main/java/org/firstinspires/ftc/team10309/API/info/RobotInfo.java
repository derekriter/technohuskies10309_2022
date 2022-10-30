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
     * The name of the servo that rotates the claw on the lift
     */
    public final static String clawRotaterName = "clawRotater";
    /**
     * The claw servo
     */
    public final static String clawName = "claw";

    /**
     * The name of the IMU in the control hub
     */
    public final static String imuName = "imu";

    public final static String liftBottomName = "liftBottom";

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
    
    public final static String camera = "webcam";
    
    public final static String pathToModel = "/storage/emulated/0/FIRST/tflitemodels" +
            "/sleevesV2.tflite";
    
    public static final String VUFORIA_KEY =
            "AXxOC5T/////AAABmSJC57yqQENwiMY04jFbMZpnfnBAf5owbyLoU53bsaoA0LpmGf8ud2gFJoOQ7ss6mmjI5vaZ2qUqUAUuKOaiP9zXMrNKsOLSi/5K///mA6hn70YKnuCSNEIS/amvBAnwgX0r9syIb9IjfcLtobMqn5Pbqea10aBH8blAjip9pNnsXeHhybkyoyMiajtOzKx79SCcwAgyvXZOikLqNPz4u1JKpwdPGj3vvTRgVhCc2Wswv13U+Wyp0ti889pKmoazuT6ByUoCl0WEuZKf9I+HZUGaa8MSjNyMMw38Zee9KfwkCYfFmBI6N7XLOp7M6DF5oeNblQMR77jvHj8+yqCTFczzvAWlX+uzXLZTdHnbhwPm";
    public static final String[] model_labels = {
            "Blue Triangle",
            "Green Circle",
            "Red Square"
    };

    /**
     * The number of ticks representing the top limit of the lift, assuming that the bottom is
     * zero, and up is negative
     */
    public final static int liftTop = -14572;
}