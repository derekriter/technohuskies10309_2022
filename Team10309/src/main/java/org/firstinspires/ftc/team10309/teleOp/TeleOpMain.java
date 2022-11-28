package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.Robot;
import org.firstinspires.ftc.team10309.API.RobotHardware;

import org.firstinspires.ftc.team10309.API.info.RobotInfo;

@TeleOp(name="TeleOpMain", group="Test")
public class TeleOpMain extends LinearOpMode {

    public RobotHardware hardware;

    private float driveSpeedMultiplier = 0.4f;

    private boolean decreaseSpeedLast;
    private boolean increaseSpeedLast;

    @Override
    public void runOpMode() {
        this.hardware = new RobotHardware(this, true);

        telemetry.addLine("Init completed");
        telemetry.update();

        waitForStart();
        this.hardware.resetLift();

        while(opModeIsActive()) {
            customLoop();
        }
    }

    public void customLoop() {
        final float liftSpeed = 1f;
        final float armPosLeft = 0.05f;
        final float armPosCenter = 0.4f;
        final float armPosRight = 0.8f;
        final float clawOpenPos = 0.35f;
        final float clawClosePos = 0.15f;

        boolean raiseLift = gamepad2.y;
        boolean lowerLift = gamepad2.a;
        boolean armLeft = gamepad2.dpad_left;
        boolean armCenter = gamepad2.dpad_up;
        boolean armRight = gamepad2.dpad_right;
        boolean openClaw = gamepad2.left_bumper;
        boolean closeClaw = gamepad2.right_bumper;
        boolean decreaseSpeed = gamepad1.left_trigger > 0.5;
        boolean increaseSpeed = gamepad1.right_trigger > 0.5;

        if(decreaseSpeed && !decreaseSpeedLast)  {
            if(driveSpeedMultiplier > 0.21) driveSpeedMultiplier -= 0.2;
        }
        if(increaseSpeed && !increaseSpeedLast) {
            if(driveSpeedMultiplier < 0.59) driveSpeedMultiplier += 0.2;
        }

        if(this.hardware.getLiftBottom().isPressed()) this.hardware.resetLift();

        if(raiseLift && this.hardware.getLift().getCurrentPosition() > RobotInfo.liftTop){
            this.hardware.getLift().setPower(-liftSpeed);
        }
        else if(lowerLift && this.hardware.getLift().getCurrentPosition() < 0
                && (this.hardware.getClawRotator().getPosition() != 0.4 || this.hardware.getLift().getCurrentPosition() <= -1100)) {
            this.hardware.getLift().setPower(liftSpeed);
        }
        else {
            this.hardware.getLift().setPower(0);
        }

        if(armLeft && this.hardware.getLift().getCurrentPosition() <= -1100) {
            this.hardware.getClawRotator().setPosition(armPosLeft);
        }
        else if(armCenter && this.hardware.getLift().getCurrentPosition() <= -1100) {
            this.hardware.getClawRotator().setPosition(armPosCenter);
        }
        else if(armRight && this.hardware.getLift().getCurrentPosition() <= -1100) {
            this.hardware.getClawRotator().setPosition(armPosRight);
        }

        if(openClaw) {
            this.hardware.getClaw().setPosition(clawOpenPos);
        }
        if(closeClaw) {
            this.hardware.getClaw().setPosition(clawClosePos);
        }

        double forward = -this.gamepad1.left_stick_y;
        double strafe = this.gamepad1.left_stick_x;
        double turn = this.gamepad1.right_stick_x;

        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        this.hardware.getFLMotor().setPower(flPower * driveSpeedMultiplier);
        this.hardware.getBRMotor().setPower(brPower * driveSpeedMultiplier);
        this.hardware.getFRMotor().setPower(frPower * driveSpeedMultiplier);
        this.hardware.getBLMotor().setPower(blPower * driveSpeedMultiplier);

        telemetry.addData("Lift Pos", this.hardware.getLift().getCurrentPosition());
        telemetry.addData("Claw Pos", this.hardware.getClaw().getPosition());
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("Drive Speed Shifter", driveSpeedMultiplier);
        telemetry.update();

        decreaseSpeedLast = decreaseSpeed;
        increaseSpeedLast = increaseSpeed;
    }
}
