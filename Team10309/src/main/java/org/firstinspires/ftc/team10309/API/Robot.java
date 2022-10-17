package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.position = new Vec2(0, 0);
        this.rotation = 0;
        this.hardware = new RobotHardware(this.opMode);

        this.calibrateEncoders();
        this.calibrateGyro();
    }

    /**
     * Recalibrates the encoders in the motors, and sets the robots position to (0, 0)
     */
    private void calibrateEncoders() {
        this.hardware.getFLMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.position = new Vec2(0, 0);
    }
    /**
     * Recalibrates the gyro, and sets the robot's rotation value back to zero. Called during in
     * the constructor during init
     */
    private void calibrateGyro() {
        this.rotation = 0;
    }

    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot
     * @param speed the speed to drive at (min: -1, max: 1)
     */
    public void drive(float inches, float speed) {
        float clampedSpeed = Math.max(Math.min(speed, 1), -1);
        int ticks = calculateTicks(inches, RobotInfo.driveTPR, RobotInfo.driveDiameter);
        calibrateEncoders();

        DcMotor fl = this.hardware.getFLMotor();
        DcMotor fr = this.hardware.getFLMotor();
        DcMotor bl = this.hardware.getFLMotor();
        DcMotor br = this.hardware.getFLMotor();

        fl.setTargetPosition(ticks);
        fr.setTargetPosition(ticks);
        bl.setTargetPosition(ticks);
        br.setTargetPosition(ticks);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(clampedSpeed);
        fr.setPower(clampedSpeed);
        bl.setPower(clampedSpeed);
        br.setPower(clampedSpeed);
    }
    /**
     * A function to move the robot left and right
     * @param inches the distance to move the robot
     */
    public void strafe(float inches) {

    }
    /**
     * A function to turn the robot
     * @param degrees the amount of degrees to turn
     */
    public void turn(float degrees) {

    }
    /**
     * A function to move the robot to a designated tile on the playing field. (0, 0) is at the
     * blue terminal on the blue side
     * @param x the x position of the tile
     * @param y the y position of the tile
     */
    public void gotoTile(float x, float y) {}

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
     * A function containing the PID algorithm
     * @param target The target value for the PID algorithm to try to match
     * @param current The current value for the PID algorithm to move towards the target
     * @param Kp The Kp modifier value
     * @param Ki The Ki modifier value
     * @param Kd The Kd modifier value
     * @return The resulting value of the PID algorithm
     */
    private float PID(double target, double current, float Kp, float Ki, float Kd) {return 0;}
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
     * @param tpr the ticks per revolution of the motor
     * @param diameter the diameter of the wheel the motor is driving
     * @return the number of ticks needed
     */
    private int calculateTicks(float targetDist, int tpr, float diameter) {
        return (int) (targetDist / tpr * diameter);
    }
}
