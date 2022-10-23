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
    
    /**
     * The name of the camera
     */
    public final static String camera = "webcam"; // CHANGE
    
    public final static String pathToModel = "/storage/emulated/0/FIRST/tflitemodels" +
            "/sleevesV2.tflite";
    
    public static final String VUFORIA_KEY =
            "AXxOC5T/////AAABmSJC57yqQENwiMY04jFbMZpnfnBAf5owbyLoU53bsaoA0LpmGf8ud2gFJoOQ7ss6mmjI5vaZ2qUqUAUuKOaiP9zXMrNKsOLSi/5K///mA6hn70YKnuCSNEIS/amvBAnwgX0r9syIb9IjfcLtobMqn5Pbqea10aBH8blAjip9pNnsXeHhybkyoyMiajtOzKx79SCcwAgyvXZOikLqNPz4u1JKpwdPGj3vvTRgVhCc2Wswv13U+Wyp0ti889pKmoazuT6ByUoCl0WEuZKf9I+HZUGaa8MSjNyMMw38Zee9KfwkCYfFmBI6N7XLOp7M6DF5oeNblQMR77jvHj8+yqCTFczzvAWlX+uzXLZTdHnbhwPm";
    public static final String[] model_labels = {
            "Blue Triangle",
            "Green Circle",
            "Red Square"
    };
}
