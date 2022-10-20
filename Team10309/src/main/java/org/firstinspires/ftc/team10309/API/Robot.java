package org.firstinspires.ftc.team10309.API;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team10309.API.info.FieldInfo;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

/**
 * A class representing the robot, all of its data, and its capabilities
 */
public class Robot implements Runnable {

    /**
     * A 2D Vector representing the robots position
     */
    private Vec2 position;

    /**
     * The op mode the robot is running
     */
    private final LinearOpMode opMode;
    /**
     * The hardware of this robot instance
     */
    private final RobotHardware hardware;
    /**
     * The thread running the update loop
     */
    private Thread loop;
    /**
     * Whether this is the final robot or robobobo
     */
    private boolean isFinal;

    public Robot(LinearOpMode opMode, float tileX, float tileY,
                 boolean isFinal) {
        this.opMode = opMode;
        this.position = new Vec2(tileX * FieldInfo.tileSize, tileY * FieldInfo.tileSize);
        this.hardware = new RobotHardware(this.opMode, isFinal);
        this.isFinal = isFinal;

        this.resetEncoders();
        this.resetGyro();
    }

    /**
     * Recalibrates the encoders in the motors
     */
    private void resetEncoders() {
        this.hardware.getFLMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /**
     * Recalibrates the gyro
     */
    private void resetGyro() {
        this.hardware.imuParams = new BNO055IMU.Parameters();
        this.hardware.imuParams.mode = BNO055IMU.SensorMode.IMU;
        this.hardware.getIMU().initialize(this.hardware.imuParams);
    }

    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot (+: forward, -: backward)
     * @param speed the speed to drive at (min: 0, max: 1)
     */
    public void drive(float inches, float speed) {
        float clampedSpeed = Math.max(Math.min(speed, 1), 0);
        int ticks = this.calculateTicks(inches);

        this.resetEncoders();

        this.hardware.getFLMotor().setTargetPosition(ticks);
        this.hardware.getFRMotor().setTargetPosition(ticks);
        this.hardware.getBLMotor().setTargetPosition(ticks);
        this.hardware.getBRMotor().setTargetPosition(ticks);

        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.hardware.getFLMotor().setPower(clampedSpeed);
        this.hardware.getFRMotor().setPower(clampedSpeed);
        this.hardware.getBLMotor().setPower(clampedSpeed);
        this.hardware.getBRMotor().setPower(clampedSpeed);

        this.waitForMotors();
    }
    /**
     * A function to move the robot left and right
     * @param inches the distance to move the robot (+: right, -: left)
     * @param speed the speed to drive at (min: 0, max: 1)
     */
    public void strafe(float inches, float speed) {
        float clampedSpeed = Math.max(Math.min(speed, 1), 0);
        int ticks = Math.round(this.calculateTicks(inches) * 1.25f);

        this.resetEncoders();

        this.hardware.getFLMotor().setTargetPosition(ticks);
        this.hardware.getFRMotor().setTargetPosition(-ticks);
        this.hardware.getBLMotor().setTargetPosition(-ticks);
        this.hardware.getBRMotor().setTargetPosition(ticks);

        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.hardware.getFLMotor().setPower(clampedSpeed);
        this.hardware.getFRMotor().setPower(clampedSpeed);
        this.hardware.getBLMotor().setPower(clampedSpeed);
        this.hardware.getBRMotor().setPower(clampedSpeed);

        this.waitForMotors();
    }
    /**
     * rotates to robot
     * @param degrees the number of degrees to turn the robot (+: clockwise, -: counterclockwise)
     * @param speed the speed to turn at (min: 0, max: 1)
     */
    public void turn(float degrees, float speed) {
        resetGyro();
        Orientation orientation = this.hardware.getIMU().getAngularOrientation(
                        AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    /**
     * Starts the update loop
     */
    public void startUpdateLoop() {
        this.loop = new Thread(this);
        this.loop.start();
    }
    /**
     * Stops the update loop
     */
    public void exitUpdateLoop() {
        this.loop.interrupt();
    }

    /**
     * DON'T USE THIS OUTSIDE OF API
     */
    public void run() {
        while(this.opMode.opModeIsActive()) {
            this.update();
        }
    }
    /**
     * A function to be called every frame of the program
     */
    private void update() {
        updateGyro();
    }

    /**
     * A function to be called in the update loop, which will handle measuring rotation from the
     * built-in gyroscope, and properly adjust the robot's rotation property
     */
    private void updateGyro() {

    }

    /**
     * Forces the program to wait until the motors are done moving
     */
    private void waitForMotors() {
        while(
            this.hardware.getFLMotor().isBusy()
            || this.hardware.getFRMotor().isBusy()
            || this.hardware.getBLMotor().isBusy()
            || this.hardware.getBRMotor().isBusy()
        ) {}
    }
    /**
     * Calculates the number of ticks needed to travel the specified distance
     * @param targetDist the number of inches you want to travel
     * @return the number of ticks needed
     */
    private int calculateTicks(float targetDist) {
        int tpr = this.isFinal ? RobotInfo.finalDriveTPR : RobotInfo.protoDriveTPR;
        float diameter = this.isFinal ? RobotInfo.finalDriveDiameter : RobotInfo.protoDriveDiameter;

        return Math.round((targetDist / (float) (diameter * Math.PI)) * tpr);
    }
}
