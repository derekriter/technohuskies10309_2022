package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.Robot;
import org.firstinspires.ftc.team10309.API.RobotHardware;

import org.firstinspires.ftc.team10309.API.info.RobotInfo;

@TeleOp(name="TeleOpMain", group="Test")
public class TeleOpMain extends LinearOpMode {

    private enum MotionDirection {
        DRIVE,
        STRAFE
    }

    public RobotHardware hardware;
    public Robot robot;

    private float driveSpeedMultiplier = 0.4f;

    private boolean decreaseSpeedLast;
    private boolean increaseSpeedLast;

    private boolean tileForwardLast;
    private boolean tileBackwardLast;
    private boolean tileLeftLast;
    private boolean tileRightLast;

    @Override
    public void runOpMode() {
        this.robot = new Robot(this, true);
        this.hardware = robot.getHardware();

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
            if(driveSpeedMultiplier < 0.61) driveSpeedMultiplier += 0.2;
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

        boolean tileForward = this.gamepad1.dpad_up;
        boolean tileBackward = this.gamepad1.dpad_down;
        boolean tileLeft = this.gamepad1.dpad_left;
        boolean tileRight = this.gamepad1.dpad_right;
        boolean tileDownshift = this.gamepad1.left_bumper;

        if(tileForward && !tileForwardLast) {
            shiftDirection(MotionDirection.DRIVE, tileDownshift ? 0.5f : 1f);
        }
        if(tileBackward && !tileBackwardLast) {
            shiftDirection(MotionDirection.DRIVE, tileDownshift ? -0.5f : -1f);
        }
        if(tileLeft && !tileLeftLast) {
            shiftDirection(MotionDirection.STRAFE, tileDownshift ? -0.5f : -1f);
        }
        if(tileRight && !tileRightLast) {
            shiftDirection(MotionDirection.STRAFE, tileDownshift ? 0.5f : 1f);
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

        tileForwardLast = tileForward;
        tileBackwardLast = tileBackward;
        tileLeftLast = tileLeft;
        tileRightLast = tileRight;
    }

    private void shiftDirection(MotionDirection direction, float distance) {
        if(distance == 0) return;
        float clampedDistance = Math.min(Math.max(distance, -1), 1);

        if(direction == MotionDirection.DRIVE) {
            robot.driveTiles(clampedDistance, driveSpeedMultiplier);
        }
        else {
            robot.strafeTiles(clampedDistance, driveSpeedMultiplier);
        }
    }
}
