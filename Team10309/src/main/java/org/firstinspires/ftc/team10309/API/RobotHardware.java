package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
     * Represents channel A of the drive odometer wheel
     */
    private DigitalChannelImpl dOdometerA;
    /**
     * Represents channel B of the drive odometer wheel
     */
    private DigitalChannelImpl dOdometerB;
    /**
     * Represents channel A of the strafe odometer wheel
     */
    private DigitalChannelImpl sOdometerA;
    /**
     * Represents channel B of the strafe odometer wheel
     */
    private DigitalChannelImpl sOdometerB;

    //Constructors for AutoOp and TeleOp

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
//
//    public DigitalChannelImpl getDOdometerA() {return this.dOdometerA;}
//    public DigitalChannelImpl getDOdometerB() {return this.dOdometerB;}
//    public DigitalChannelImpl getSOdometerA() {return this.sOdometerA;}
//    public DigitalChannelImpl getSOdometerB() {return this.sOdometerB;}

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

//        this.dOdometerA = hardwareMap.get(DigitalChannelImpl.class, RobotInfo.dOdometerAName);
//        this.dOdometerB = hardwareMap.get(DigitalChannelImpl.class, RobotInfo.dOdometerBName);
//        this.sOdometerA = hardwareMap.get(DigitalChannelImpl.class, RobotInfo.sOdometerAName);
//        this.sOdometerB = hardwareMap.get(DigitalChannelImpl.class, RobotInfo.sOdometerBName);
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
//
//        this.dOdometerA.setMode(DigitalChannel.Mode.INPUT);
//        this.dOdometerB.setMode(DigitalChannel.Mode.INPUT);
//        this.sOdometerA.setMode(DigitalChannel.Mode.INPUT);
//        this.sOdometerB.setMode(DigitalChannel.Mode.INPUT);
    }
}
