package org.firstinspires.ftc.team10309.API;

/**
 * A class representing the robot, all of its data, and its capabilities
 */
public abstract class Robot {

    /**
     * API ONLY! A 2D Vector representing the robots position
     */
    private static Vec2 position;
    /**
     * API ONLY! A float representing the robots rotation in degrees
     */
    private static float rotation;

    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot
     *               +: forward
     *               -: backward
     */
    public static void drive(float inches) {}
    /**
     * A function to move the robot left and right
     * @param inches the distance to move the robot
     *               +: right
     *               -: left
     */
    public static void strafe(float inches) {}
    /**
     * A function to turn the robot
     * @param degrees the amount of degrees to turn
     *                +: clockwise
     *                -: counter-clockwise
     */
    public static void turn(float degrees) {}
    /**
     * A function to move the robot to a designated tile on the playing field. (0, 0) is at the
     * blue terminal on the blue side
     * @param x the x position of the tile
     * @param y the y position of the tile
     */
    public static void gotoTile(float x, float y) {}

    /**
     * API ONLY! A function to handle the rotation of the odometer wheels, and update the robot's
     * position and rotation accordingly
     * @param degLeft The amount of degrees the left odometer wheel has turned since the last check
     * @param degRight The amount of degrees the right odometer wheel has turned since the last
     *                 check
     * @param degBack The amount of degrees the back wheel has turned since the last check
     */
    private static void handleOdometry(double degLeft, double degRight, double degBack) {}

    /**
     * API ONLY! A function containing the PID algorithm
     * @param target The target value for the PID algorithm to try to match
     * @param current The current value for the PID algorithm to move towards the target
     * @param Kp The Kp modifier value
     * @param Ki The Ki modifier value
     * @param Kd The Kd modifier value
     * @return The resulting value of the PID algorithm
     */
    private static double PID(double target, double current, float Kp, float Ki, float Kd) {return 0;}
}
