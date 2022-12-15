package org.firstinspires.ftc.team10309.API;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.team10309.API.info.RobotInfo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotHardware {

    private final boolean isFinal;

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor lift;

    private Servo clawRotater;
    private Servo claw;

    private BNO055IMU imu;
    public BNO055IMU.Parameters imuParams;
    
    private WebcamName camera;
    private TouchSensor liftBottom;

    private DcMotorEx driveOdo;
    private DcMotorEx strafeOdo;

    private VoltageSensor voltageSensor;

    private final HardwareMap hw;

    public RobotHardware(LinearOpMode opMode, boolean isFinal) {
        this.isFinal = isFinal;
        this.hw = opMode.hardwareMap;

        this.mapHardware(opMode.hardwareMap);
        this.configHardware();
    }

    public DcMotor getFLMotor() {return this.flMotor;}
    public DcMotor getFRMotor() {return this.frMotor;}
    public DcMotor getBLMotor() {return this.blMotor;}
    public DcMotor getBRMotor() {return this.brMotor;}
    public DcMotor getLift() {return this.lift;}

    public Servo getClawRotator() {return this.clawRotater;}
    public Servo getClaw() {return this.claw;}

    public WebcamName getCamera() {return this.camera;}
    public BNO055IMU getIMU() {return this.imu;}
    public TouchSensor getLiftBottom() {return this.liftBottom;}

    public DcMotorEx getDriveOdo() {return this.driveOdo;}
    public DcMotorEx getStrafeOdo() {return this.strafeOdo;}

    public VoltageSensor getVoltageSensor() {return this.voltageSensor;}

    private void mapHardware(HardwareMap hardwareMap) {
        this.flMotor = hardwareMap.get(DcMotor.class, RobotInfo.flMotorName);
        this.frMotor = hardwareMap.get(DcMotor.class, RobotInfo.frMotorName);
        this.blMotor = hardwareMap.get(DcMotor.class, RobotInfo.blMotorName);
        this.brMotor = hardwareMap.get(DcMotor.class, RobotInfo.brMotorName);

        this.imu = hardwareMap.get(BNO055IMU.class, RobotInfo.imuName);
        this.imuParams = new BNO055IMU.Parameters();

        for(VoltageSensor vs : hardwareMap.voltageSensor) {
            this.voltageSensor = vs;
            break;
        }

        if(this.isFinal) {
            this.lift = hardwareMap.get(DcMotor.class, RobotInfo.liftName);
            this.clawRotater = hardwareMap.get(Servo.class, RobotInfo.clawRotaterName);
            this.claw = hardwareMap.get(Servo.class, RobotInfo.clawName);
            this.liftBottom = hardwareMap.get(TouchSensor.class, RobotInfo.liftBottomName);

            this.driveOdo = hardwareMap.get(DcMotorEx.class, RobotInfo.driveOdoName);
            this.strafeOdo = hardwareMap.get(DcMotorEx.class, RobotInfo.strafeOdoName);
        }

        this.camera = hardwareMap.get(WebcamName.class, RobotInfo.camera);
    }

    private void configHardware() {
        if(isFinal) {
            this.brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            this.claw.setDirection(Servo.Direction.REVERSE);
            this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.driveOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.strafeOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else {
            this.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            this.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        this.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.imuParams.mode = BNO055IMU.SensorMode.IMU;
        this.imu.initialize(this.imuParams);
    }
    public int getTfodMonitorViewID() {
        return this.hw.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", this.hw.appContext.getPackageName());
    }

    public void resetEncoders() {
        this.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetIMU() {
        this.imuParams = new BNO055IMU.Parameters();
        this.imuParams.mode = BNO055IMU.SensorMode.IMU;
        this.getIMU().initialize(this.imuParams);
    }
    public void resetLift() {
        if(!this.liftBottom.isPressed()) {
            this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.lift.setPower(0.4);
        }
        while(!this.liftBottom.isPressed()) {}

        this.lift.setPower(0);

        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setTargetPosition(this.lift.getCurrentPosition() - 50);
        this.lift.setPower(0.2);

        while(this.lift.isBusy()) {}

        this.lift.setPower(0);

        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
