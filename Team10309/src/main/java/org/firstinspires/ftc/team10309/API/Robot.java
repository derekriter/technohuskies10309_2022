package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team10309.API.info.FieldInfo;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

import java.util.ArrayList;

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

//    public void turn(float degrees, float speed, double precision) {
//        this.hardware.resetIMU();
//        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        this.hardware.getFLMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.hardware.getFRMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.hardware.getBRMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.hardware.getBLMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
////        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
////        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
////        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
////        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        // See which motors should be backwards, which should be forwards.
//        // More compatible: make an array of each motor in an order in specified in a comment.
//        // Then negate each motor accordingly.
//        angles = this.hardware.getIMU().getAngularOrientation(
//                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        turn(degrees, speed, precision, 0.1, 1/500f, 1/50f,
//                new double[] {angles.secondAngle-degrees,
//                        angles.secondAngle-degrees});
//    }
//    private void turn(double degrees, double speed, double precision, double Kp,
//                      double Ki, double Kd, double[] trend) {
//        if (!opMode.opModeIsActive()) {
//            return;
//        }
//        // angles.thirdAngle = detected angle
//        // degrees = target angle
//
//        // "base case"
//        // trend is misleading, it should be sumOfErrors or something. trend is just for abriviation.
//        angles = this.hardware.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,
//                AngleUnit.DEGREES);
//        double angle = angles.secondAngle;
//        //angle = degrees > 0 ? (angle > 0 ? angle : angle + 360) : (angle < 0 ? angle :
//        //     angle - 360);
////        if (degrees > 0 && angle < 0) {
////            angle = angle + 360;
////        }
////        if (degrees < 0 && angle > 0) {
////            angle = angle - 360;
////        }
//
//        double err = angle - degrees;
//        if (Math.abs(err) <= precision) {
//            this.hardware.getFLMotor().setPower(0);
//            this.hardware.getFRMotor().setPower(0);
//            this.hardware.getBRMotor().setPower(0);
//            this.hardware.getBLMotor().setPower(0);
//            // left side reverse on bobo
//            // right side reverse on frank
////            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
////            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
////            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
////            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            // see above coment on direction setting... (in prev. turn method and here)
//            return;
//        }
//
//        // logic - dont forget to multiply double final = -(p + i + d); bc we want to cancel out the final (var) error
//        double proportional = err * Kp;
//        double integral = DoubleStream.of(trend).sum() * Ki;
//        double derivative = (err - trend[trend.length - 1]) * Kd;
//        double total = -(proportional + integral + derivative) * speed;
//
//        // #nestedterneries r awesum.
//        double basePower = 0.05;
//        if (total > 0) {
//            total = total < basePower ? total + basePower : total;
//        }
//        if (total < 0) {
//            total = total > basePower ? total - basePower : total;
//        }
//
//        // ROBOBOBO: LEFT 2 NEGATE
//        // Final: ???
//        hardware.getFRMotor().setPower(total);
//        hardware.getFLMotor().setPower(-total);
//        hardware.getBLMotor().setPower(-total);
//        hardware.getBRMotor().setPower(total);
//
//        this.opMode.telemetry.addData("Angle: ", "" + angles.firstAngle);
//        this.opMode.telemetry.addData("Second Angle: ", "" + angles.secondAngle);
//        this.opMode.telemetry.addData("Third Angle: ", "" + angles.thirdAngle);
//        this.opMode.telemetry.addData("I: ", integral + "D: " + derivative + "T: " + total);
//        this.opMode.telemetry.update();
//
//        // recursive updates
//        double[] trendI = new double[trend.length + 1];
//        for (int i = 0; i < trend.length; i++) trendI[i] = trend[i];
//        trendI[trendI.length - 1] = err;
//
//        turn(degrees, speed, precision, Kp, Ki, Kd, trendI);
//    }

    public void turn(float degrees, double aPrecision) throws InterruptedException {
        double batteryVoltage = this.hardware.getVoltageSensor().getVoltage();
        double powerAdjustment = 13f / batteryVoltage;
//        double powerAdjustment = 1;

        //reset IMU, so the start angle is 0
        this.hardware.resetIMU();

        //make sure you are using RUN_USING_ENCODER, so you can control motors with power
        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //the target angle is the inverse of the inputed target
        double target = -degrees;

        //PID controller parameters for angle correction
        double aKp, aKi, aKd;

//        double k = Math.abs(target);
//            Kp = 30/k;
//            Ki = 0.6/k;
//            Kd = k + 10;
//            aKp = 0.014*k;
//            aKi = 0.00035*k;
//            aKd = 0.01*k;

        //tune Kp, Ki, and Kd by the target degrees

//        if(Math.abs(target) > 130) {
//            aKp = 0.3;
//            aKi = 0.001;
//            aKd = 7;
//        }
//        else if(Math.abs(target) > 110) {
//            aKp = 0.2;
//            aKi = 0.001;
//            aKd = 6;
//        }
//        else if(Math.abs(target) > 90) {
//            aKp = 0.3;
//            aKi = 0.001;
//            aKd = 7;
//        }
        if(Math.abs(target) > 75) {
            aKp = 0.3;
            aKi = 0.0002;
            aKd = 5;
        }
//        else if(Math.abs(target) > 60) {
//            aKp = 0.4;
//            aKi = 0.004;
//            aKd = 6;
//        }
//        else if(Math.abs(target) > 45) {
//            aKp = 0.4;
//            aKi = 0.005;
//            aKd = 5;
//        }
        else if(Math.abs(target) > 30) {
            aKp = 0.4;
            aKi = 0.0005;
            aKd = 4;
        }
        else if(Math.abs(target) > 15) {
            aKp = 0.4;
            aKi = 0.006;
            aKd = 2;
        }
        else {
            aKp = 0.5;
            aKi = 0.02;
            aKd = 3;
        }

        Orientation angles = this.hardware.getIMU().getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //calculate inital error
        double angle = angles.secondAngle;
        double err = angle - target;   //
        double multiplier = 0.03; //power gain for angle correction

        //keeping track of trend for I and adding current start error
        ArrayList<Double> trend = new ArrayList<>();
        trend.add(err);

        //keeps track of whether the robot can break out of the turn loop
        boolean beforeTarget = true;

        while (beforeTarget && this.opMode.opModeIsActive()) {
            //get angle from IMU
            angles = this.hardware.getIMU().getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            //calc PID
            err = angles.secondAngle - target;
            double P = err;
            double I = trend.stream().reduce(0d, Double::sum);
            double D = err - trend.get(trend.size() - 1);
            double correction = (aKp * P) + (aKi * I) + (aKd * D);

            
            
            
            //apply multiplier
            correction *= multiplier;

            //clamp minimum speed
            if (correction > 0) {
                correction = Math.max(0.00001, correction);
            }
            else if (correction < 0) {
                correction = Math.min(correction, -0.00001);
            }

            //clamp maximum speed
            double aSpeed = 0.3;
            if (correction < 0 && correction < -aSpeed) {
                correction = -aSpeed;
            }
            else if (correction > 0 && correction > aSpeed) {
                correction = aSpeed;
            }
            //add error to trend
            trend.add(err);

            //move motors
            this.hardware.getFLMotor().setPower(correction * powerAdjustment);
            this.hardware.getFRMotor().setPower(-correction * powerAdjustment);
            this.hardware.getBLMotor().setPower(correction * powerAdjustment);
            this.hardware.getBRMotor().setPower(-correction * powerAdjustment);

            //recalculator angles
            angles = this.hardware.getIMU().getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            //test for breakout condition
            if(target < 0) {
                beforeTarget = angles.secondAngle - target > aPrecision;
            }
            else {
                beforeTarget = angles.secondAngle - target < -aPrecision;
            }

        }

        //stop motors
        this.hardware.getFLMotor().setPower(0);
        this.hardware.getFRMotor().setPower(0);
        this.hardware.getBLMotor().setPower(0);
        this.hardware.getBRMotor().setPower(0);

        //recalculate angles for a final time
        angles = this.hardware.getIMU().getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //output final telemetry
        this.opMode.telemetry.addData("Target angle", degrees);
        this.opMode.telemetry.addData("Final angle", angles.secondAngle);
        this.opMode.telemetry.update();
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
        if(inches == 0) return;
        
        this.hardware.resetIMU();
        double batteryVoltage = this.hardware.getVoltageSensor().getVoltage();
        double powerAdjustment = 12.6f / batteryVoltage;

        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double target = -inches;
        double inchesPerTick = RobotInfo.odoDiameter * Math.PI / 1440;    //inch per tick
        double err = 0 - target;   //initial err = initial position 0 - target
        final double multiplier = 0.05;  //motor power gain
        final double precision = 0.1;   //distance precision in inches

        //PID controller parameters Kp, Ki, Kd
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;

        // PID controller parameters for angle correction
        double aKp = 0;
        double aKi = 0;
        double aKd = 0;

//        double k = Math.abs(target);
//            Kp = 30/k;
//            Ki = 0.6/k;
//            Kd = k + 10;
//            aKp = 0.014*k;
//            aKi = 0.00035*k;
//            aKd = 0.01*k;

        if(Math.abs(target)>80) {
            Kp = 0.42;
            Ki = 0.009;
            Kd = 85.0;
            aKp = 0.5;
            aKi = 0.012;
            aKd = 0.5;
        }
        else if(Math.abs(target)>70) {
            Kp = 0.2;
            Ki = 0.001;
            Kd = 15.0;
            aKp = 0.5;
            aKi = 0.012;
            aKd = 0.5;
        }
        else if(Math.abs(target)>60) {
            Kp = 0.55;
            Ki = 0.011;
            Kd = 75.0;
            aKp = 0.5;
            aKi = 0.012;
            aKd = 0.5;
        }
        else if(Math.abs(target)>50) {
            Kp = 0.65;
            Ki = 0.013;
            Kd = 70.0;
            aKp = 0.5;
            aKi = 0.012;
            aKd = 0.5;
        }
        else if(Math.abs(target)>40) {
            Kp = 0.8;
            Ki = 0.016;
            Kd = 60.0;
            aKp = 0.5;
            aKi = 0.012;
            aKd = 0.5;
        }
        else if(Math.abs(target)>30) {
            Kp = 1;
            Ki = 0.02;
            Kd = 50.0;
            aKp = 0.5;
            aKi = 0.012;
            aKd = 0.5;
        }
        else if(Math.abs(target)>20) {
            Kp = 1.3;
            Ki = 0.025;
            Kd = 35.0;
            aKp = 0.4;
            aKi = 0.01;
            aKd = 0.4;
        }
        else if(Math.abs(target)>10) {
            Kp = 1;
            Ki = 0.01;
            Kd = 20.0;
            aKp = 0.2;
            aKi = 0.05;
            aKd = 0.2;
        }
        else {
            Kp = 1.0;
            Ki = 0.01;
            Kd = 20.0;
            aKp = 0.1;
            aKi = 0.003;
            aKd = 0.1;
        }

        DcMotorEx enc = this.hardware.getDriveOdo();
        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArrayList<Double> trend = new ArrayList<>();
        trend.add(err);

        Orientation angles = this.hardware.getIMU().getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double angle = angles.secondAngle;
        double aErr = angle - 0;   //target = 0 for going straight, aErr = measured angle - target
        double aMultiplier = 0.05; //power gain for angle correction
        double aPrecision = 0.1;   //angle precision in degrees

        double aSpeed = 0;   // angle correction cap

        ArrayList<Double> a_trend = new ArrayList<>();
        a_trend.add(aErr);

        while(Math.abs(enc.getCurrentPosition()*inchesPerTick - target) > precision && this.opMode.opModeIsActive()) {
            angles = this.hardware.getIMU().getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            err = enc.getCurrentPosition() * inchesPerTick - target;
            aErr = angles.secondAngle - 0;   //target = 0 degree for going straight
            double sum = trend.stream().reduce(0d, Double::sum);
            double a_sum = a_trend.stream().reduce(0d, Double::sum);
            double corr = Kp * err + Ki * sum + Kd * (-trend.get(trend.size() - 1) + err);
            double angleCorr = aKp * aErr + aKi * a_sum + aKd * (-a_trend.get(a_trend.size() - 1) + aErr);

            corr *= multiplier;
            angleCorr *= aMultiplier;
            if (angleCorr > 0) {
                angleCorr = Math.max(0.01, angleCorr);
            } else if (angleCorr < 0) {
                angleCorr = Math.min(angleCorr, -0.01);
            }

            if (corr < 0 && corr > -0.1) {
                corr = -0.1;
            } else if (corr > 0 && corr < 0.1) {
                corr = 0.1;
            } else if (corr < 0 && corr < -speed) {
                corr = -speed;
            } else if (corr > 0 && corr > speed) {
                corr = speed;
            }

            
            corr = corr* powerAdjustment;
            aSpeed = 0.4 * Math.abs(corr);
            if (angleCorr < 0 && angleCorr < -aSpeed) {
                angleCorr = -aSpeed;
            } else if (angleCorr > 0 && angleCorr > aSpeed) {
                angleCorr = aSpeed;
            }

            if (Math.abs(aErr) < aPrecision) {
                this.hardware.getFLMotor().setPower(corr);
                this.hardware.getFRMotor().setPower(corr);
                this.hardware.getBLMotor().setPower(corr);
                this.hardware.getBRMotor().setPower(corr);
            } else {
                this.hardware.getFLMotor().setPower(corr + angleCorr);
                this.hardware.getFRMotor().setPower(corr - angleCorr);
                this.hardware.getBLMotor().setPower(corr + angleCorr);
                this.hardware.getBRMotor().setPower(corr - angleCorr);
            }
            trend.add(err);
            a_trend.add(aErr);
        }

        this.hardware.getFLMotor().setPower(0);
        this.hardware.getFRMotor().setPower(0);
        this.hardware.getBLMotor().setPower(0);
        this.hardware.getBRMotor().setPower(0);

        this.opMode.telemetry.addData("Final Tick Value", enc.getCurrentPosition());
        this.opMode.telemetry.update();
    }
    public void strafeOdo(float inches, float speed) {
        this.hardware.resetIMU();
        this.hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double batteryVoltage = this.hardware.getVoltageSensor().getVoltage();
        double powerAdjustment = 12.6f / batteryVoltage;

        double target = - inches;
        double err = 0 - target;
        final double multiplier = 0.05; //motor power gain
        double precision = 0.1;  //distance precision in inches
        double Kp = 1;   //PID controller parameters Kp, Ki, Kd
        double Ki = 0.02;
        double Kd = 55;
    
        double aKp = 0.4;          // PID controller parameters for angle correction
        double aKi = 0.01;
        double aKd = 0.4;
//
//        if(Math.abs(target)>80) {
//            Kp = 0.42;
//            Ki = 0.009;
//            Kd = 85.0;
//            aKp = 0.5;
//            aKi = 0.012;
//            aKd = 0.5;
//        }
//        else if(Math.abs(target)>70) {
//            Kp = 0.2;
//            Ki = 0.001;
//            Kd = 15.0;
//            aKp = 0.5;
//            aKi = 0.012;
//            aKd = 0.5;
//        }
//        else if(Math.abs(target)>60) {
//            Kp = 0.55;
//            Ki = 0.011;
//            Kd = 75.0;
//            aKp = 0.5;
//            aKi = 0.012;
//            aKd = 0.5;
//        }
//        else if(Math.abs(target)>50) {
//            Kp = 0.65;
//            Ki = 0.013;
//            Kd = 70.0;
//            aKp = 0.5;
//            aKi = 0.012;
//            aKd = 0.5;
//        }
//        else if(Math.abs(target)>40) {
//            Kp = 0.8;
//            Ki = 0.016;
//            Kd = 60.0;
//            aKp = 0.5;
//            aKi = 0.012;
//            aKd = 0.5;
//        }
//        else
        if(Math.abs(target)>30) {
            Kp = 0.4;
            Ki = 0.02;
            Kd = 20.0;
            aKp = 0.5;
            aKi = 0.012;
            aKd = 0.5;
        }
        else if(Math.abs(target)>20) {
            Kp = 0.4;
            Ki = 0.025;
            Kd = 20.0;
            aKp = 0.4;
            aKi = 0.01;
            aKd = 0.4;
        }
        else if(Math.abs(target)>10) {
            Kp = 1;
            Ki = 0.01;
            Kd = 20.0;
            aKp = 0.2;
            aKi = 0.05;
            aKd = 0.2;
        }
        else {
            Kp = 1.0;
            Ki = 0.01;
            Kd = 20.0;
            aKp = 0.1;
            aKi = 0.003;
            aKd = 0.1;
        }

        double inchesPerTick = RobotInfo.odoDiameter * Math.PI / 1440;    //inch per tick

        DcMotorEx enc = this.hardware.getStrafeOdo();
        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArrayList<Double> trend = new ArrayList<>();
        trend.add(err);

        Orientation angles = this.hardware.getIMU().getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double angle = angles.secondAngle;
        double aErr = angle - 0;   //angle err = measured angle - target, target = 0
        double aMultiplier = 0.05; //power gain for angle correction
        double aPrecision = 0.1;     //angle precision in degrees
        double aSpeed = 0;  // angle correction cap

        ArrayList<Double> a_trend = new ArrayList<>();
        a_trend.add(aErr);

        while(Math.abs(enc.getCurrentPosition() * inchesPerTick - target) > precision && this.opMode.opModeIsActive()) {
            angles = this.hardware.getIMU().getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            err = enc.getCurrentPosition() * inchesPerTick - target;
            aErr = angles.secondAngle - 0;

            double sum = trend.stream().reduce(0d, Double::sum);
            double a_sum = a_trend.stream().reduce(0d, Double::sum);
            double corr = Kp * err + Ki * sum + Kd * (-trend.get(trend.size() - 1) + err);
            double angleCorr = aKp * aErr + aKi * a_sum + aKd * (-a_trend.get(a_trend.size() - 1) + aErr);

            corr *= multiplier;
            angleCorr *= aMultiplier;

            if (angleCorr > 0) {
                angleCorr = Math.max(0.01, angleCorr);
            } else if (angleCorr < 0) {
                angleCorr = Math.min(angleCorr, -0.01);
            }

            if (corr < 0 && corr > -0.1) {
                corr = -0.1;
            } else if (corr > 0 && corr < 0.1) {
                corr = 0.1;
            } else if (corr < 0 && corr < -speed) {
                corr = -speed;
            } else if (corr > 0 && corr > speed) {
                corr = speed;
            }

            aSpeed = 0.4 * Math.abs(corr);
            if (angleCorr < 0 && angleCorr < -aSpeed) {
                angleCorr = -aSpeed;
            } else if (angleCorr > 0 && angleCorr > aSpeed) {
                angleCorr = aSpeed;
            }

            if(Math.abs(aErr) < aPrecision) {
                this.hardware.getFLMotor().setPower(corr);
                this.hardware.getFRMotor().setPower(-corr);
                this.hardware.getBLMotor().setPower(-corr);
                this.hardware.getBRMotor().setPower(corr);
            }
            else {
                this.hardware.getFLMotor().setPower(corr + angleCorr);
                this.hardware.getFRMotor().setPower(-corr + angleCorr);
                this.hardware.getBLMotor().setPower(-corr - angleCorr);
                this.hardware.getBRMotor().setPower(corr - angleCorr);

            }

            trend.add(err);
            a_trend.add(aErr);
        }
        this.hardware.getFLMotor().setPower(0);
        this.hardware.getFRMotor().setPower(0);
        this.hardware.getBLMotor().setPower(0);
        this.hardware.getBRMotor().setPower(0);

        this.opMode.telemetry.addData("Final Tick Value", enc.getCurrentPosition());
        this.opMode.telemetry.update();

//        this.turn((float) -aErr, 0.1f, 1);
    }


    public void teleValues() {
        // IMU
        // HORIZ &
        // DRIVE odo
        this.hardware.getDriveOdo().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getStrafeOdo().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (true) {
            this.opMode.telemetry.addData("IMU", this.hardware.getIMU().getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);

            this.opMode.telemetry.addData("Drive ODO", this.hardware.getDriveOdo().getCurrentPosition());
            this.opMode.telemetry.addData("Horiz ODO", this.hardware.getStrafeOdo().getCurrentPosition());
            opMode.telemetry.update();
        }
    }
}
