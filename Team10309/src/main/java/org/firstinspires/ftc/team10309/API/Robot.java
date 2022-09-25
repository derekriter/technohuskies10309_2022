package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A class representing the robot, all of its data, and its capabilities
 */
public class Robot {

    /**
     * A 2D Vector representing the robots position
     */
    private Vec2 position;
    /**
     * A float representing the robots rotation in degrees
     */
    private float rotation;

    /**
     * The op mode the robot is running
     */
    private final OpMode opMode;

    /**
     *An object representing the front left motor
     */
    private final DcMotor flMotor;
    /**
     * An object representing the front right motor
     */
    private final DcMotor frMotor;
    /**
     * An object representing the back left motor
     */
    private final DcMotor blMotor;
    /**
     * An object representing the back right motor
     */
    private final DcMotor brMotor;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        this.position = new Vec2(0, 0);
        this.rotation = 0;

        this.flMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotInfo.flMotorName);
        this.frMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotInfo.frMotorName);
        this.blMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotInfo.blMotorName);
        this.brMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotInfo.brMotorName);

        this.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * A function to be called every frame of the program, to handle tasks
     */
    public void update() {
        this.handleOdometry(0, 0, 0);
    }

    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot
     */
    public void drive(float inches) {
        this.flMotor.setPower(1);
        this.frMotor.setPower(1);
        this.blMotor.setPower(1);
        this.brMotor.setPower(1);
    }
    /**
     * A function to move the robot left and right
     * @param inches the distance to move the robot
     */
    public void strafe(float inches) {}
    /**
     * A function to turn the robot
     * @param degrees the amount of degrees to turn
     */
    public void turn(float degrees) {}
    /**
     * A function to move the robot to a designated tile on the playing field. (0, 0) is at the
     * blue terminal on the blue side
     * @param x the x position of the tile
     * @param y the y position of the tile
     */
    public void gotoTile(float x, float y) {}

    /**
     * A function to handle the rotation of the odometer wheels, and update the robot's
     * position and rotation accordingly
     * @param degLeft The amount of degrees the left odometer wheel has turned since the last check
     * @param degRight The amount of degrees the right odometer wheel has turned since the last
     *                 check
     * @param degBack The amount of degrees the back wheel has turned since the last check
     */
    private void handleOdometry(double degLeft, double degRight, double degBack) {}

    /**
     * A function containing the PID algorithm
     * @param target The target value for the PID algorithm to try to match
     * @param current The current value for the PID algorithm to move towards the target
     * @param Kp The Kp modifier value
     * @param Ki The Ki modifier value
     * @param Kd The Kd modifier value
     * @return The resulting value of the PID algorithm
     */
    private double PID(double target, double current, float Kp, float Ki, float Kd) {return 0;}
}
