package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team10309.API.info.FieldInfo;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

import java.util.stream.DoubleStream;

/**
 * A class representing the robot, all of its data, and its capabilities
 */
public class Robot {
    /**
     * The op mode the robot is running
     */
    private final LinearOpMode opMode;
    /**
     * The hardware of this robot instance
     */
    private final RobotHardware hardware;
    /**
     * Whether this is the final robot or robobobo
     */
    private boolean isFinal;

    public Robot(LinearOpMode opMode,
                 boolean isFinal) {
        
        this.opMode = opMode;
        this.hardware = new RobotHardware(this.opMode, isFinal);
        this.isFinal = isFinal;
        
        this.hardware.resetEncoders();
        this.hardware.resetIMU();
    }

    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot (+: forward, -: backward)
     * @param speed the speed to drive at (min: 0, max: 1)
     */
    public void drive(float inches, float speed) {
        float clampedSpeed = Math.max(Math.min(speed, 1), 0);
        int ticks = this.calculateTicks(inches);

        this.hardware.resetEncoders();

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

        this.hardware.resetEncoders();

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
     * Drives the number of tiles
     * @param tiles the number of tiles
     * @param speed the speed (0 - 1)
     */
    public void driveTiles(float tiles, float speed) {
        this.drive(tiles * FieldInfo.tileSize, speed);
    }
    /**
     * strafes the number of tiles
     * @param tiles the number of tiles
     * @param speed the speed (0 - 1)
     */
    public void strafeTiles(float tiles, float speed) {
        this.strafe(tiles * FieldInfo.tileSize, speed);
    }

    private Orientation angles;
    public void turn(float degrees, float speed, double precision) {
        this.hardware.resetIMU();
        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getFLMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hardware.getFRMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hardware.getBRMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hardware.getBLMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        // See which motors should be backwards, which should be forwards.
        // More compatible: make an array of each motor in an order in specified in a comment.
        // Then negate each motor accordingly.
        angles = this.hardware.getIMU().getAngularOrientation(
                        AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        turn(degrees, speed, precision, 0.1, 1/500, 1/50,
                new double[] {angles.secondAngle-degrees,
                        angles.secondAngle-degrees});
    }

    /**
     * Helper method for public void turn.
     * ROBOBOBO: LEFT 2 NEGATE
     * Final: theoretically the same as robobobo...
     */
    private void turn(double degrees, double speed, double precision, double Kp,
                        double Ki, double Kd, double[] trend) {
        if (!opMode.opModeIsActive()) {
            return;
        }
        // angles.thirdAngle = detected angle
        // degrees = target angle

        // "base case"
        // trend is misleading, it should be sumOfErrors or something. trend is just for abriviation.
        angles = this.hardware.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,
                AngleUnit.DEGREES);
        double angle = angles.secondAngle;
        //angle = degrees > 0 ? (angle > 0 ? angle : angle + 360) : (angle < 0 ? angle :
        //     angle - 360);
        if (degrees > 0 && angle < 0) {
            angle = angle + 360;
        }
        if (degrees < 0 && angle > 0) {
            angle = angle - 360;
        }

        double err = angle - degrees;
        if (Math.abs(err) <= precision) {
            this.hardware.getFLMotor().setPower(0);
            this.hardware.getFRMotor().setPower(0);
            this.hardware.getBRMotor().setPower(0);
            this.hardware.getBLMotor().setPower(0);
            // left side reverse on bobo
            // right side reverse on frank
//            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        // see above coment on direction setting... (in prev. turn method and here)
            return;
        }

        // logic - dont forget to multiply double final = -(p + i + d); bc we want to cancel out the final (var) error
        double proportional = err * Kp;
        double integral = DoubleStream.of(trend).sum() * Ki;
        double derivative = (err - trend[trend.length - 1]) * Kd;
        double total = -(proportional + integral + derivative) * speed;
        
        // #nestedterneries r awesum.
        double basePower = 0.05;
        if (total > 0) {
            total = total < basePower ? total + basePower : total;
        }
        if (total < 0) {
            total = total > basePower ? total - basePower : total;
        }

        // ROBOBOBO: LEFT 2 NEGATE
        // Final: ???
        hardware.getFRMotor().setPower(total);
        hardware.getFLMotor().setPower(-total);
        hardware.getBLMotor().setPower(-total);
        hardware.getBRMotor().setPower(total);

        this.opMode.telemetry.addData("Angle: ", "" + angles.firstAngle);
        this.opMode.telemetry.addData("Second Angle: ", "" + angles.secondAngle);
        this.opMode.telemetry.addData("Third Angle: ", "" + angles.thirdAngle);
        this.opMode.telemetry.addData("I: ",  + integral + "D: " + derivative + "T: " + total);
        this.opMode.telemetry.update();

        // recursive updates
        double[] trendI = new double[trend.length + 1];
        for (int i = 0; i < trend.length; i++) trendI[i] = trend[i];
        trendI[trendI.length - 1] = err;

        turn(degrees, speed, precision, Kp, Ki, Kd, trendI);
    }
    
    /**
     * Forces the program to wait until the motors are done moving
     */
    private void waitForMotors() {
        while(
            (
                this.hardware.getFLMotor().isBusy()
                || this.hardware.getFRMotor().isBusy()
                || this.hardware.getBLMotor().isBusy()
                || this.hardware.getBRMotor().isBusy()
            )
            && this.opMode.opModeIsActive()
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

    public SleeveDetect sleeveDetect;

    public void initDetect() {
        sleeveDetect = new SleeveDetect(this.opMode.telemetry, this.hardware.getCamera(),
                this.hardware);
        sleeveDetect.init();
    }
    public SleeveDetect.SignalState scanSleeve() {

        SleeveDetect.SignalState ss = sleeveDetect.scan();

        return ss;
    }

    public SleeveDetect.SignalState initAndScan() {
        initDetect();
        return scanSleeve();
    }

    /**
     * Returns the internally stored RobotHardware
     * @return RobotHardware for robot instance
     */
    public RobotHardware getHardware() {return this.hardware;}
}
