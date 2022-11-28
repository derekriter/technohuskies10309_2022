package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.*;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

@TeleOp(name="TeleOpMain")
public class TeleOpMain extends LinearOpMode {

    public RobotHardware hardware;
    public Robot robot;
    public ManipulatorController manipulatorController;

    private float driveSpeedMultiplier = 0.4f;
    private boolean decreaseSpeedLast = false;
    private boolean increaseSpeedLast = false;

    private boolean driverMode = false;
    private boolean changeDriverModeLast = false;

    @Override
    public void runOpMode() {
        this.robot = new Robot(this, true);
        this.hardware = this.robot.getHardware();
        this.manipulatorController = new ManipulatorController(this.hardware, this);

        telemetry.addLine("Init completed");
        telemetry.update();

        waitForStart();
        this.hardware.resetLift();

        while(opModeIsActive()) customLoop();
    }

    public void customLoop() {
        //
        //MANIPULATOR
        //

        final float liftSpeed = 1f;
        final float armPosLeft = 0.05f;
        final float armPosCenter = 0.4f;
        final float armPosRight = 0.8f;
        final float clawOpenPos = 0.35f;
        final float clawClosePos = 0.15f;

        float lift = -gamepad2.right_stick_y;
        boolean presetGround = gamepad2.b;
        boolean presetLow = gamepad2.a;
        boolean presetMiddle = gamepad2.x;
        boolean presetHigh = gamepad2.y;

        boolean armLeft = gamepad2.dpad_left;
        boolean armCenter = gamepad2.dpad_up;
        boolean armRight = gamepad2.dpad_right;

        boolean openClaw = gamepad2.left_bumper;
        boolean closeClaw = gamepad2.right_bumper;

        if(this.hardware.getLiftBottom().isPressed()) this.hardware.resetLift();

        if(presetGround) manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND, false);
        else if(presetLow) manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.LOW, false);
        else if(presetMiddle) manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.MIDDLE, false);
        else if(presetHigh) manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.HIGH, false);

        if(lift > 0 && this.hardware.getLift().getCurrentPosition() > RobotInfo.liftTop) {
            this.hardware.getLift().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardware.getLift().setPower(-liftSpeed * lift);
        }
        else if(lift < 0 && this.hardware.getLift().getCurrentPosition() < 0
                && (this.hardware.getClawRotator().getPosition() != 0.4
                || this.hardware.getLift().getCurrentPosition() <= -1100)) {
            this.hardware.getLift().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardware.getLift().setPower(-liftSpeed * lift);
        }
        else if(!this.hardware.getLift().isBusy()) {
            this.hardware.getLift().setPower(0);
        }

        if(armLeft && this.hardware.getLift().getCurrentPosition() <= -726) {
            this.hardware.getClawRotator().setPosition(armPosLeft);
        }
        else if(armCenter && this.hardware.getLift().getCurrentPosition() <= -726) {
            this.hardware.getClawRotator().setPosition(armPosCenter);
        }
        else if(armRight && this.hardware.getLift().getCurrentPosition() <= -726) {
            this.hardware.getClawRotator().setPosition(armPosRight);
        }

        if(openClaw) {
            this.hardware.getClaw().setPosition(clawOpenPos);
        }
        if(closeClaw) {
            this.hardware.getClaw().setPosition(clawClosePos);
        }

        //
        //DRIVER
        //

        boolean decreaseSpeed = gamepad1.left_trigger > 0.5;
        boolean increaseSpeed = gamepad1.right_trigger > 0.5;

        boolean changeDriverMode = gamepad1.start;

        double forward = -this.gamepad1.left_stick_y;
        double strafe = this.gamepad1.left_stick_x;
        double turn = this.gamepad1.right_stick_x;

        if(decreaseSpeed && !decreaseSpeedLast)  {
            if(driveSpeedMultiplier > 0.21) driveSpeedMultiplier -= 0.2;
        }
        if(increaseSpeed && !increaseSpeedLast) {
            if(driveSpeedMultiplier < 0.61) driveSpeedMultiplier += 0.2;
        }

        if(changeDriverMode && !changeDriverModeLast) driverMode = !driverMode;

        double flPower, frPower, blPower, brPower;
        //one direction driving
        if(!driverMode) {
            boolean priority = Math.max(Math.abs(forward), Math.abs(strafe)) == Math.abs(forward);

            flPower = (priority ? forward : strafe) + turn;
            frPower = (priority ? forward : -strafe) - turn;
            blPower = (priority ? forward : -strafe) + turn;
            brPower = (priority ? forward : strafe) - turn;
        }
        //omni-directional driving
        else {
            flPower = forward + strafe + turn;
            frPower = forward - strafe - turn;
            blPower = forward - strafe + turn;
            brPower = forward + strafe - turn;
        }
        this.hardware.getFLMotor().setPower(flPower * driveSpeedMultiplier);
        this.hardware.getBRMotor().setPower(brPower * driveSpeedMultiplier);
        this.hardware.getFRMotor().setPower(frPower * driveSpeedMultiplier);
        this.hardware.getBLMotor().setPower(blPower * driveSpeedMultiplier);

        telemetry.addData("Lift Pos", this.hardware.getLift().getCurrentPosition());
        telemetry.addData("Claw Pos", this.hardware.getClaw().getPosition());
        telemetry.addData("Drive Speed Shifter", driveSpeedMultiplier);
        telemetry.update();

        //
        //AFTER FRAME
        //

        decreaseSpeedLast = decreaseSpeed;
        increaseSpeedLast = increaseSpeed;

        changeDriverModeLast = changeDriverMode;
    }
}
