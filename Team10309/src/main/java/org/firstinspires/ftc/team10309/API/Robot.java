package org.firstinspires.ftc.team10309.API;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.android.dx.cf.cst.ConstantPoolParser;
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

    public Robot(LinearOpMode pOpMode,
                 boolean isFinal) {
        
        this.opMode = pOpMode;
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
     * Drives the number of tiles
     * @param tiles the number of tiles
     * @param speed the speed (0 - 1)
     * @param extraInches extra inchhes after driveTiles so it won't stop start stop start.
     */
    public void driveTiles(float tiles, float speed, float extraInches) {
        this.drive(tiles * FieldInfo.tileSize + extraInches, speed);
    }
    
    /**
     * strafes the number of tiles
     * @param tiles the number of tiles
     * @param speed the speed (0 - 1)
     */
    public void strafeTiles(float tiles, float speed) {
        this.strafe(tiles * FieldInfo.tileSize, speed);
    }
    /**
     * strafes the number of tiles
     * @param tiles the number of tiles
     * @param speed the speed (0 - 1)
     * @param extraInches extra inchhes after strafTiles so it won't stop start stop start.
     *
     */
    public void strafeTiles(float tiles, float speed, float extraInches) {
        this.strafe(tiles * FieldInfo.tileSize + extraInches, speed);
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
        ) {
            this.opMode.telemetry.addData("Wait for motors FL",
                    this.hardware.getFLMotor().isBusy());
            this.opMode.telemetry.addData("Wait for motors FR",
                    this.hardware.getFRMotor().isBusy());
            this.opMode.telemetry.addData("Wait for motors BL",
                    this.hardware.getBLMotor().isBusy());
            this.opMode.telemetry.addData("Wait for motors BR",
                    this.hardware.getBRMotor().isBusy());
            this.opMode.telemetry.update();
        }
        
        this.opMode.telemetry.addData("Out of Motor Waiting","Done");
        this.opMode.telemetry.update();
        
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
//        sleeveDetect = new SleeveDetect(this.opMode.telemetry, this.hardware.getCamera(),
//                this.hardware);
        sleeveDetect.init();
    }
    public SleeveDetect.SignalState scanSleeve() throws InterruptedException {

        SleeveDetect.SignalState ss = sleeveDetect.scan();

        return ss;
    }

    public SleeveDetect.SignalState initAndScan() throws InterruptedException {
        initDetect();
        return scanSleeve();
    }

    /**
     * Returns the internally stored RobotHardware
     * @return RobotHardware for robot instance
     */
    public RobotHardware getHardware() {return this.hardware;}
    
    
    
    
    /**
     * Goes to absolute position, quickest way possible... discontinue
     */
//    private Location currentLocation;
//    public void goToPosition(Location location, double speed) {
//        if (location.equals(currentLocation)) {return;}
//
//        double clampedSpeed = Math.max(Math.min(speed, 1), 0);
//        double inchesX = currentLocation.x - location.x;
//        double inchesY = currentLocation.y - location.y;
//        int distanceX = calculateTicks((float)inchesX);
//        int distanceY = this.calculateTicks((float)inchesY);
//        int distance =
//                this.calculateTicks((float)Math.sqrt(distanceX*distanceX + distanceY*distanceY));
//
//
//        this.hardware.resetEncoders();
//
//        this.hardware.getFLMotor().setTargetPosition(ticks);
//        this.hardware.getFRMotor().setTargetPosition(ticks);
//        this.hardware.getBLMotor().setTargetPosition(ticks);
//        this.hardware.getBRMotor().setTargetPosition(ticks);
//
//        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        this.hardware.getFLMotor().setPower(clampedSpeed);
//        this.hardware.getFRMotor().setPower(clampedSpeed);
//        this.hardware.getBLMotor().setPower(clampedSpeed);
//        this.hardware.getBRMotor().setPower(clampedSpeed);
//
//        this.waitForMotors();
//
//
//        currentLocation = location;
//    }
    
    /**
     * Goes to relative position
     */
    public void driveOdo(double inches, double speed, double[] trend) {
        opMode.hardwareMap.get(DcMotor.class, "front left").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
        this.opMode.hardwareMap.get(DcMotor.class, "front right").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.hardwareMap.get(DcMotor.class, "back left").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.hardwareMap.get(DcMotor.class, "back right").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        double precision = 500;
        double Kp = 0.5;
        double Ki = 1/200;
        double Kd = 1/50;
        
        if (!opMode.opModeIsActive()) {
            return;
        }
    
        double err =
                opMode.hardwareMap.get(DcMotorEx.class, "front left").getCurrentPosition()-inches*360/1.2
                ;
        if (Math.abs(err) <= precision) {
            this.hardware.getFLMotor().setPower(0);
            this.hardware.getFRMotor().setPower(0);
            this.hardware.getBRMotor().setPower(0);
            this.hardware.getBLMotor().setPower(0);
            return;
        }
    
        double proportional = err * Kp;
        double integral = DoubleStream.of(trend).sum() * Ki;
        double derivative = (err - trend[trend.length - 1]) * Kd;
        double total = (proportional + integral + derivative) * speed;
    
        double basePower = 0.05;
        if (total > 0) {
            total = total < basePower ? total + basePower : total;
        }
        if (total < 0) {
            total = total > basePower ? total - basePower : total;
        }
        
        this.opMode.telemetry.addData("Error: ", err);
        this.opMode.telemetry.addData("P", proportional);
        this.opMode.telemetry.addData("I", integral);
        this.opMode.telemetry.addData("D", derivative);
        this.opMode.telemetry.addData("Odo Reading", opMode.hardwareMap.get(DcMotorEx.class,
                "front left").getCurrentPosition());
        this.opMode.telemetry.update();
    
        // recursive updates
        double[] trendI = new double[trend.length + 1];
        for (int i = 0; i < trend.length; i++) trendI[i] = trend[i];
        trendI[trendI.length - 1] = err;
    
        hardware.getFRMotor().setPower(total);
        hardware.getFLMotor().setPower(-total);
        hardware.getBLMotor().setPower(total);
        hardware.getBRMotor().setPower(-total);
        
        this.waitForMotors();
        driveOdo(inches, speed, trendI);
    };
    
    class Location {
        private double x;
        private double y;
    
        public void setX(double x) {
            this.x = x;
        }
        public void setY(double y) {
            this.y = y;
        }
        public void setXTiles(double xTiles) {
            this.xTiles = xTiles;
        }
        public void setYTiles(double yTiles) {
            this.yTiles = yTiles;
        }
    
        public void addX(double x) {
            this.x += x;
        }
        public void addY(double y) {
            this.y += y;
        }
        public void addXTiles(double xTiles) {
            this.xTiles += xTiles;
        }
        public void addYTiles(double yTiles) {
            this.yTiles += yTiles;
        }
        
        public void subtractX(double x) {
            this.x -= x;
        }
        public void subtractY(double y) {
            this.y -= y;
        }
        public void subtractXTiles(double xTiles) {
            this.xTiles -= xTiles;
        }
        public void subtractYTiles(double yTiles) {
            this.yTiles -= yTiles;
        }
        
        public void multiplyX(double x) {
            this.x *= x;
        }
        public void multiplyY(double y) {
            this.y *= y;
        }
        public void multiplyXTiles(double xTiles) {
            this.xTiles *= xTiles;
        }
        public void multiplyYTiles(double yTiles) {
            this.yTiles *= yTiles;
        }
    
        public void divideX(double x) {
            this.x /= x;
        }
        public void divideY(double y) {
            this.y /= y;
        }
        public void divideXTiles(double xTiles) {
            this.xTiles /= xTiles;
        }
        public void divideYTiles(double yTiles) {
            this.yTiles /= yTiles;
        }
        
        private double xTiles;
        private double yTiles;
    
        public double getX() {
            return x;
        }
        public double getY() {
            return y;
        }
        public double getXTiles() {
            return xTiles;
        }
        public double getYTiles() {
            return yTiles;
        }
        
        
        Location(float xTiles, float yTiles) {
            this.x = xTiles/FieldInfo.tileSize;
            this.y = yTiles/FieldInfo.tileSize;
            this.xTiles = xTiles;
            this.yTiles = yTiles;
        }
        Location(double x, double y) {
            this.x = x;
            this.y = y;
            this.xTiles = x*FieldInfo.tileSize;
            this.yTiles = yTiles*FieldInfo.tileSize;
        }
        
        public boolean equals(@Nullable Location obj) {
            if (obj == null) return false;
            
            return this.x == obj.x && this.y == obj.y;
        }
    }
}
