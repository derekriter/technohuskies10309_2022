package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private final LinearOpMode opMode;

    private final RobotHardware hardware;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.position = new Vec2(0, 0);
        this.rotation = 0;

        this.hardware = new RobotHardware(this.opMode);
    }

    /**
     * A function to be called every frame of the program, to handle tasks
     */
    public void update() {
        this.handleOdometry(0, 0);
        this.handleGyroscope(0);
    }

    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot
     */
    public void drive(float inches) {
        this.hardware.getFLMotor().setPower(1);
        this.hardware.getFRMotor().setPower(1);
        this.hardware.getBLMotor().setPower(1);
        this.hardware.getBRMotor().setPower(1);
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
     * position accordingly
     * @param drive The amount of degrees the drive odometer wheel has turned since the last check
     * @param strafe The amount of degrees the strafe odometer wheel has turned since the last check
     */
    private void handleOdometry(float drive, float strafe) {
        if(drive == 0 && strafe == 0) return;

        float driveDist = calcDist(drive);
        float strafeDist = calcDist(strafe);
    }

    /**
     * Updates the robot's rotation based off of the gyro's angle
     * @param angle The gyro's angle
     */
    private void handleGyroscope(float angle) {

    }
    /**
     * A function to find the distance a wheel moved based off of it's rotation
     * @param deg the amount of degrees the wheel has turned
     * @return the distance the wheel has moved in inches
     */
    private float calcDist(float deg) {
        return deg / 360 * RobotInfo.odometerCirc;
    }

    /**
     * A function containing the PID algorithm
     * @param target The target value for the PID algorithm to try to match
     * @param current The current value for the PID algorithm to move towards the target
     * @param Kp The Kp modifier value
     * @param Ki The Ki modifier value
     * @param Kd The Kd modifier value
     * @return The resulting value of the PID algorithm
     */
    private float PID(double target, double current, float Kp, float Ki, float Kd) {return 0;}
}
