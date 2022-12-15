package org.firstinspires.ftc.team10309.API.info;

public abstract class RobotInfo {

    public final static String flMotorName = "front left";
    public final static String frMotorName = "front right";
    public final static String blMotorName = "back left";
    public final static String brMotorName = "back right";
    public final static String liftName = "lift";
    public final static String clawRotaterName = "clawRotater";
    public final static String clawName = "claw";

    public final static String imuName = "imu";

    public final static String liftBottomName = "liftBottom";
    public final static int liftTop = -9533;

    public final static int protoDriveTPR = 1120;
    public final static float protoDriveDiameter = 4;
    public final static int finalDriveTPR = 560;
    public final static float finalDriveDiameter = 4;
    
    public final static String camera = "webcam";
    public final static String pathToModel = "/storage/emulated/0/FIRST/tflitemodels/sleevesV2.tflite";
    public static final String VUFORIA_KEY = "AXxOC5T/////AAABmSJC57yqQENwiMY04jFbMZpnfnBAf5owbyLoU53bsaoA0LpmGf8ud2gFJoOQ7ss6mmjI5vaZ2qUqUAUuKOaiP9zXMrNKsOLSi/5K///mA6hn70YKnuCSNEIS/amvBAnwgX0r9syIb9IjfcLtobMqn5Pbqea10aBH8blAjip9pNnsXeHhybkyoyMiajtOzKx79SCcwAgyvXZOikLqNPz4u1JKpwdPGj3vvTRgVhCc2Wswv13U+Wyp0ti889pKmoazuT6ByUoCl0WEuZKf9I+HZUGaa8MSjNyMMw38Zee9KfwkCYfFmBI6N7XLOp7M6DF5oeNblQMR77jvHj8+yqCTFczzvAWlX+uzXLZTdHnbhwPm";
    public static final String[] model_labels = {
            "Blue Triangle",
            "Green Circle",
            "Red Square"
    };

    public final static float odoDiameter = 1.464f;
    public final static String driveOdoName = "driveOdo";
    public final static String strafeOdoName = "strafeOdo";
}