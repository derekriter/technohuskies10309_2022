package org.firstinspires.ftc.team10309.API;

import android.sax.StartElementListener;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team10309.API.info.FieldInfo;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

import java.util.ArrayList;
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
    private final boolean isFinal;
    
    public Robot(LinearOpMode pOpMode,
                 boolean isFinal) {
        this.opMode = pOpMode;
        this.hardware = new RobotHardware(this.opMode, isFinal);
        this.isFinal = isFinal;
    }
    
    public void drive(float inches, float speed) {
        drive(inches, speed, true);
    }
    /**
     * A function to move the robot forward and backward
     * @param inches the distance to move the robot (+: forward, -: backward)
     * @param speed the speed to drive at (min: 0, max: 1)
     */
    public void drive(float inches, float speed, boolean waitForMotors) {
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
        
        if(waitForMotors) this.waitForMotors();
    }
    
    public void strafe(float inches, float speed) {
        strafe(inches, speed, true);
    }
    /**
     * A function to move the robot left and right
     * @param inches the distance to move the robot (+: right, -: left)
     * @param speed the speed to drive at (min: 0, max: 1)
     */
    public void strafe(float inches, float speed, boolean waitForMotors) {
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
        
        if(waitForMotors) this.waitForMotors();
    }
    
    public void driveTiles(float tiles, float speed) {
        driveTiles(tiles, speed, true);
    }
    /**
     * Drives the number of tiles
     * @param tiles the number of tiles (+: forward, -: backward)
     * @param speed the speed (0 - 1)
     */
    public void driveTiles(float tiles, float speed, boolean waitForMotors) {
        this.drive(tiles * FieldInfo.tileSize, speed, waitForMotors);
    }
    
    public void driveTiles(float tiles, float speed, float extraInches) {
        driveTiles(tiles, speed, extraInches, true);
    }
    /**
     * Drives the number of tiles
     * @param tiles the number of tiles  (+: forward, -: backward)
     * @param speed the speed (0 - 1)
     * @param extraInches extra inches to add the distance.
     */
    public void driveTiles(float tiles, float speed, float extraInches, boolean waitForMotors) {
        this.drive(tiles * FieldInfo.tileSize + extraInches, speed, waitForMotors);
    }
    
    public void strafeTiles(float tiles, float speed) {
        strafeTiles(tiles, speed, true);
    }
    /**
     * Drives the number of tiles
     * @param tiles the number of tiles  (+: right, -: left)
     * @param speed the speed (0 - 1)
     */
    public void strafeTiles(float tiles, float speed, boolean waitForMotors) {
        this.strafe(tiles * FieldInfo.tileSize, speed, waitForMotors);
    }
    
    public void strafeTiles(float tiles, float speed, float extraInches) {
        strafeTiles(tiles, speed, extraInches, true);
    }
    /**
     * Drives the number of tiles
     * @param tiles the number of tiles  (+: right, -: left)
     * @param speed the speed (0 - 1)
     * @param extraInches extra inches to add the distance.
     */
    public void strafeTiles(float tiles, float speed, float extraInches, boolean waitForMotors) {
        this.strafe(tiles * FieldInfo.tileSize + extraInches, speed, waitForMotors);
    }
    
    private Orientation angles;
    
    /**
     * turns using PID
     * @param degrees target degrees (only use positive)
     * @param speed the speed
     * @param precision how close it has to be to the target degrees before it stops !! DON'T SET
     *                 TO ZERO !!
     */
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
        turn(degrees, speed, precision, 0.1, 1/500f, 1/50f,
                new double[] {angles.secondAngle-degrees,
                        angles.secondAngle-degrees});
    }
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
//        if (degrees > 0 && angle < 0) {
//            angle = angle + 360;
//        }
//        if (degrees < 0 && angle > 0) {
//            angle = angle - 360;
//        }
        
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
        this.opMode.telemetry.addData("I: ", integral + "D: " + derivative + "T: " + total);
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
    public void waitForMotors() {
        while(
                (
                        this.hardware.getFLMotor().isBusy()
                                || this.hardware.getFRMotor().isBusy()
                                || this.hardware.getBLMotor().isBusy()
                                || this.hardware.getBRMotor().isBusy()
                )
                        && this.opMode.opModeIsActive()
        );
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
    
    private SleeveDetect sleeveDetect;
    
    public void initDetect() {
        sleeveDetect = new SleeveDetect(this.opMode.telemetry, this.hardware.getCamera(),
                this.hardware);
        sleeveDetect.init();
    }
    public SleeveDetect.SignalState scanSleeve() throws InterruptedException {
        return sleeveDetect.scan();
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
    
    public void driveOdo(float inches, float speed) {
        this.hardware.getIMU();
        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double target = -inches * 360 / RobotInfo.odoDiameter;
        
        double err = -target;
        final double multiplier = 0.00017;
        final double precision = 15;
        final double Kp = 0.6;
        final double Ki = 0.1;
        final double Kd = 0.12;
        
        DcMotorEx enc = this.hardware.getDriveOdo();
        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        ArrayList<Double> trend = new ArrayList<>();
        trend.add(err);
        
        Orientation angles = this.hardware.getIMU().getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        
        double angle = angles.secondAngle;
        double aErr = 0 - angle;
        double aMultiplier = 0.2;
        double aPrecision = 0.1;
        double aKp = 0.1;    //0.5  0.4
        double aKi = 0.01;   //0.1  0.01
        double aKd = 0;    //0.4  0.4
        double aSpeed = 0;
        
        ArrayList<Double> a_trend = new ArrayList<>();
        a_trend.add(aErr);
//        enc.setDirection(DcMotorSimple.Direction.REVERSE);
        while(Math.abs(enc.getCurrentPosition() - target) > precision && this.opMode.opModeIsActive()) {
            this.opMode.telemetry.addData("Position", enc.getCurrentPosition());
            this.opMode.telemetry.addData("Target", target);
            this.opMode.telemetry.addData("Breakout", Math.abs(enc.getCurrentPosition() - target));
            angles = this.hardware.getIMU().getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            err = enc.getCurrentPosition() - target;
            aErr = 0 - angles.secondAngle;
            this.opMode.telemetry.addData("IMU Angle", angles.secondAngle);
            this.opMode.telemetry.addData("aErr", aErr);
            
            this.opMode.telemetry.addData("Angle Error", aErr);
            
            double sum = trend.stream().reduce(0d, Double::sum);
            double a_sum = a_trend.stream().reduce(0d, Double::sum);
            double corr = Kp * err + Ki * sum + Kd * (-trend.get(trend.size() - 1) + err);
            aSpeed = 0.4 * corr;
            double turnpid = aKp * aErr + aKi * a_sum + aKd * (-a_trend.get(a_trend.size() - 1) + aErr);
            
            
            corr *= multiplier;
            turnpid *= aMultiplier;
            if (corr < 0 && corr > -0.1) {
                corr = -0.1;
            }
            else if (corr > 0 && corr < 0.1) {
                corr = 0.1;
            }
            else if (corr < 0 && corr < -speed) {
                corr = -speed;
            }
            else if (corr > 0 && corr > speed) {
                corr = speed;
            }
            
            // lower limit: Math.abs(corr*0.2)
            // upper limti: math.abs(corr*0.4)
//            if (turnpid < 0 && turnpid > -0.1) {
//                turnpid = -Math.abs(corr * 0.01);
//            }
//            else if (turnpid > 0 && turnpid < 0.1) {
//                turnpid = Math.abs(corr * 0.01);
//            }
            else if (turnpid < 0 && turnpid < -aSpeed) {
                turnpid = -aSpeed;
            }
            else if (turnpid > 0 && turnpid > aSpeed) {
                turnpid = aSpeed;
            }
            
            this.opMode.telemetry.addData("Angle Correction", turnpid);
//            corr *= -1;
//            turnpid *= -1;
            if(Math.abs(aErr) < aPrecision) {
                this.hardware.getFLMotor().setPower(corr);
                this.hardware.getFRMotor().setPower(corr);
                this.hardware.getBLMotor().setPower(corr);
                this.hardware.getBRMotor().setPower(corr);
            }
            else {
//                if (err > 0) {
//                    this.hardware.getFLMotor().setPower(corr - turnpid);
//                    this.hardware.getFRMotor().setPower(corr + turnpid);
//                    this.hardware.getBLMotor().setPower(corr - turnpid);
//                    this.hardware.getBRMotor().setPower(corr + turnpid);
//                }
                    this.hardware.getFLMotor().setPower(corr + turnpid);
                    this.hardware.getFRMotor().setPower(corr - turnpid);
                    this.hardware.getBLMotor().setPower(corr + turnpid);
                    this.hardware.getBRMotor().setPower(corr - turnpid);
            }
            
            if (trend.size() <= 5) {
                trend.add(err);
            } else {
                trend.remove(0);
                trend.add(err);
            }
//            if (a_trend.size() <= 5) {
//                trend.add(aErr);
//            } else {
//                trend.remove(0);
//                trend.add(aErr);
//            }
            this.opMode.telemetry.update();
        }

        this.turn((float) aErr, 0.1f, aPrecision);
    }
    public void strafeOdo(float inches, float speed) {
        this.hardware.getIMU();
        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double target = -inches * 360 / RobotInfo.odoDiameter;
        
        double err = -target;
        final double multiplier = 0.00017;
        final double precision = 500;
        final double Kp = 0.6;
        final double Ki = 0.1;
        final double Kd = 0.12;
        
        DcMotorEx enc = this.hardware.getStrafeOdo();
        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        ArrayList<Double> trend = new ArrayList<>();
        trend.add(err);
        
        Orientation angles = this.hardware.getIMU().getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        
        double angle = angles.secondAngle;
        double aErr = 0 - angle;
        double aMultiplier = 0.00005;
        double aPrecision = 1;
        double aKp = 0.4;
        double aKi = aKp / 5;
        double aKd = 0.2;
        double aSpeed = 0;
        
        ArrayList<Double> a_trend = new ArrayList<>();
        a_trend.add(aErr);
//        enc.setDirection(DcMotorSimple.Direction.REVERSE);
        while(Math.abs(enc.getCurrentPosition() - target) > precision && this.opMode.opModeIsActive()) {
            this.opMode.telemetry.addData("Position", enc.getCurrentPosition());
            this.opMode.telemetry.addData("Target", target);
            this.opMode.telemetry.addData("Breakout", Math.abs(enc.getCurrentPosition() - target));
            angles = this.hardware.getIMU().getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            err = enc.getCurrentPosition() - target;
            aErr = 0 - angles.secondAngle;
            
            this.opMode.telemetry.addData("Angle Error", aErr);
            
            double sum = trend.stream().reduce(0d, Double::sum);
            double a_sum = a_trend.stream().reduce(0d, Double::sum);
            double corr = Kp * err + Ki * sum + Kd * (-trend.get(trend.size() - 1) + err);
            aSpeed = 0.4 * corr;
            double turnpid = aKp * aErr + aKi * a_sum + aKd * (-a_trend.get(a_trend.size() - 1) + aErr);
            
            
            corr *= multiplier;
            turnpid *= aMultiplier;
            if (corr < 0 && corr > -0.1) {
                corr = -0.1;
            }
            else if (corr > 0 && corr < 0.1) {
                corr = 0.1;
            }
            else if (corr < 0 && corr < -speed) {
                corr = -speed;
            }
            else if (corr > 0 && corr > speed) {
                corr = speed;
            }
            
            // lower limit: Math.abs(corr*0.2)
            // upper limti: math.abs(corr*0.4)
            if (turnpid < 0 && turnpid > -0.1) {
                turnpid = -Math.abs(corr * 0.2);
            }
            else if (turnpid > 0 && turnpid < 0.1) {
                turnpid = Math.abs(corr * 0.2);
            }
            else if (turnpid < 0 && turnpid < -aSpeed) {
                turnpid = -aSpeed;
            }
            else if (turnpid > 0 && turnpid > aSpeed) {
                turnpid = aSpeed;
            }
            
            this.opMode.telemetry.addData("Angle Correction", turnpid);
//            corr *= -1;
//            turnpid *= -1;
            if(Math.abs(aErr) < aPrecision) {
                this.hardware.getFLMotor().setPower(corr);
                this.hardware.getFRMotor().setPower(-corr);
                this.hardware.getBLMotor().setPower(-corr);
                this.hardware.getBRMotor().setPower(corr);
            }
            else {
                if (err > 0) {
                    this.hardware.getFLMotor().setPower(corr + turnpid);
                    this.hardware.getFRMotor().setPower(-corr + turnpid);
                    this.hardware.getBLMotor().setPower(-corr + turnpid);
                    this.hardware.getBRMotor().setPower(corr + turnpid);
                }
                else {
                    this.hardware.getFLMotor().setPower(corr - turnpid);
                    this.hardware.getFRMotor().setPower(-corr - turnpid);
                    this.hardware.getBLMotor().setPower(-corr - turnpid);
                    this.hardware.getBRMotor().setPower(corr - turnpid);
                }
            }
            
            if (trend.size() <= 5) {
                trend.add(err);
            } else {
                trend.remove(0);
                trend.add(err);
            }
            if (a_trend.size() <= 5) {
                trend.add(aErr);
            } else {
                trend.remove(0);
                trend.add(aErr);
            }
            this.opMode.telemetry.update();
        }
        
        this.turn((float) -aErr, 0.1f, 1);
    }
}
