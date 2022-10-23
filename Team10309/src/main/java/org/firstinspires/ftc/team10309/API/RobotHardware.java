package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * A class containing the robot's hardware map and configuration. Used for both AutoOp and TeleOp
 */
public class RobotHardware {

    /**
     * Represents the front left motor
     */
    private DcMotor flMotor;
    /**
     * Represents the front right motor
     */
    private DcMotor frMotor;
    /**
     * Represents the back left motor
     */
    private DcMotor blMotor;
    /**
     * Represents the back right motor
     */
    private DcMotor brMotor;

    /**
     * Represents the drive odometer wheel
     */
    private DigitalChannelImpl dOdometer;
    /**
     * Represents the strafe odometer wheel
     */
    private DigitalChannelImpl sOdometer;
    
    private WebcamName camera;
    //Constructors for AutoOp and TeleOp
    public HardwareMap hardwareMap;
    //TeleOp
    public RobotHardware(OpMode opMode) {
        this.mapHardware(opMode.hardwareMap);
        this.configHardware();
    }
    //AutoOp
    public RobotHardware(LinearOpMode opMode) {
        this.mapHardware(opMode.hardwareMap);
        this.configHardware();
    }

    public DcMotor getFLMotor() {return this.flMotor;}
    public DcMotor getFRMotor() {return this.frMotor;}
    public DcMotor getBLMotor() {return this.blMotor;}
    public DcMotor getBRMotor() {return this.brMotor;}

    public DigitalChannelImpl getDOdometer() {return this.dOdometer;}
    public DigitalChannelImpl getSOdometer() {return this.sOdometer;}
    
    public WebcamName getCamera() {return this.camera;}
    /**
     * Called to init all the values of the motors. Made a seperate function, so I don't have to
     * write it twice, once for each constructor
     * @param hardwareMap The hardware map of the inputed opmode
     */
    private void mapHardware(HardwareMap hardwareMap) {
        this.flMotor = hardwareMap.get(DcMotor.class, RobotInfo.flMotorName);
        this.frMotor = hardwareMap.get(DcMotor.class, RobotInfo.frMotorName);
        this.blMotor = hardwareMap.get(DcMotor.class, RobotInfo.blMotorName);
        this.brMotor = hardwareMap.get(DcMotor.class, RobotInfo.brMotorName);
//
//        this.dOdometer = hardwareMap.get(DigitalChannelImpl.class, RobotInfo.dOdometerName);
//        this.sOdometer = hardwareMap.get(DigitalChannelImpl.class, RobotInfo.sOdometerName);
        this.camera = hardwareMap.get(WebcamName.class, RobotInfo.camera);
        this.hardwareMap = hardwareMap;
    }

    /**
     * Applies settings to the hardware (ex. setting the motor direction)
     */
    private void configHardware() {
        this.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public int getTfodMonitorViewID() {
        return this.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
    }
}
