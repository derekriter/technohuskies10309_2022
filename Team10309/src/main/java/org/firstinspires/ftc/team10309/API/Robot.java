package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
     * Used for decoding encoder values into degrees
     */
    private final RotaryDecoder decoder;

    /**
     * The thread running the update loop
     */
    private Thread loop;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.position = new Vec2(0, 0);
        this.rotation = 0;

        this.hardware = new RobotHardware(this.opMode);
        this.decoder = new RotaryDecoder();
    }

    /**
     * Starts the update loop for the program
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
     * DON'T USE THIS OUTSIDE OF API
     */
    public void run() {
        while(this.opMode.opModeIsActive()) {
            this.update();
        }
    }
    /**
     * A function to be called every frame of the program, to handle tasks
     */
    private void update() {
        updateOdometry();
        updateGyro();
    }

    private void updateOdometry() {
        float strafeDeg = this.decoder.decodeToDeg(this.hardware.getSOdometerA().getState(),
                this.hardware.getSOdometerB().getState(), RotaryDecoder.OdoType.STRAFE, this.opMode);

        this.opMode.telemetry.addData("Strafe Deg", strafeDeg);
        this.opMode.telemetry.addData("Strafe A", this.hardware.getSOdometerA().getState());
        this.opMode.telemetry.addData("Strafe B", this.hardware.getSOdometerB().getState());

        this.opMode.telemetry.update();
    }
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
}
