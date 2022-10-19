package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
     * A float representing the robots rotation in degrees
     */
    private float rotation;

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

    public Robot(LinearOpMode opMode, float tileX, float tileY, float startRotation) {
        this.opMode = opMode;
        this.position = new Vec2(tileX * FieldInfo.tileSize, tileY * FieldInfo.tileSize);
        this.rotation = startRotation;
        this.hardware = new RobotHardware(this.opMode);

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
     * Recalibrates the gyro, and sets the robot's rotation value back to zero. Called during in
     * the constructor during init
     */
    private void resetGyro() {
        this.rotation = 0;
    }

    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot
     * @param speed the speed to drive at (min: -1, max: 1)
     */
    public void drive(float inches, float speed) {
        float clampedSpeed = Math.max(Math.min(speed, 1), -1);
        int ticks = this.calculateTicks(inches, RobotInfo.driveTPR, RobotInfo.driveDiameter);
        this.opMode.telemetry.addData("Ticks", ticks);
        this.opMode.telemetry.update();
        try {
            Thread.sleep(1000);
        }
        catch(Exception e) {e.printStackTrace();}

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
        ) {
            this.opMode.telemetry.addData("FL Encoder Pos",
                    this.hardware.getFLMotor().getCurrentPosition());
            this.opMode.telemetry.update();
        }
    }
    /**
     * Calculates the number of ticks needed to travel the specified distance
     * @param targetDist the number of inches you want to travel
     * @param tpr the ticks per revolution of the motor
     * @param diameter the diameter of the wheel the motor is driving
     * @return the number of ticks needed
     */
    private int calculateTicks(float targetDist, int tpr, float diameter) {
        this.opMode.telemetry.addData("targetDist", targetDist);
        this.opMode.telemetry.addData("tpr", tpr);
        this.opMode.telemetry.addData("diameter", diameter);
        return (int) (targetDist / ((diameter * Math.PI))) * tpr;
    }
}
