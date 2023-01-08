package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.*;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleOp Main")
public class TeleOpMain extends LinearOpMode {

    public RobotHardware hardware;
    public Robot robot;
    public ManipulatorController manipulatorController;

    private float driveSpeedMultiplier = 0.4f;
    private boolean decreaseSpeed = false;
    private boolean increaseSpeed = false;
    private boolean decreaseSpeedLast = false;
    private boolean increaseSpeedLast = false;

    private boolean driverMode = false;
    private boolean changeDriverMode = false;
    private boolean changeDriverModeLast = false;

    private boolean shiftForward = false;
    private boolean shiftBackward = false;
    private boolean shiftLeft = false;
    private boolean shiftRight = false;
    private boolean shiftForwardLast = false;
    private boolean shiftBackwardLast = false;
    private boolean shiftLeftLast = false;
    private boolean shiftRightLast = false;
    private final List<Float[]> shiftSteps = new ArrayList<>();
    private boolean currentlyShifting = false;
    private Thread tileShiftThread;

    @Override
    public void runOpMode() {
        this.robot = new Robot(this, true);
        this.hardware = this.robot.getHardware();
        this.manipulatorController = new ManipulatorController(this.hardware, this);

        telemetry.addLine("Init completed");
        telemetry.update();

        waitForStart();
        this.hardware.resetLift();

        while(opModeIsActive()) {
            driver();
            manipulator();
            telemetry();
            afterFrame();
        }
    }

    private void driver() {
        decreaseSpeed = gamepad1.left_trigger > 0.5;
        increaseSpeed = gamepad1.right_trigger > 0.5;

        changeDriverMode = gamepad1.right_bumper;

        double forward = -this.gamepad1.left_stick_y;
        double strafe = this.gamepad1.left_stick_x;
        double turn = this.gamepad1.right_stick_x;

        if(decreaseSpeed && !decreaseSpeedLast)  {
            if(driveSpeedMultiplier > 0.21) driveSpeedMultiplier -= 0.2;
        }
        if(increaseSpeed && !increaseSpeedLast) {
            if(driveSpeedMultiplier < 0.39) driveSpeedMultiplier += 0.2;
        }

        if(changeDriverMode && !changeDriverModeLast) driverMode = !driverMode;

        if((forward != 0 || strafe != 0 || turn != 0) && tileShiftThread != null) {
            if(tileShiftThread.isAlive()) tileShiftThread.interrupt();
        }

        shiftForward = gamepad1.dpad_up;
        shiftBackward = gamepad1.dpad_down;
        shiftLeft = gamepad1.dpad_left;
        shiftRight = gamepad1.dpad_right;

        if(shiftForward && !shiftForwardLast) {
            shiftSteps.add(new Float[] {(gamepad1.left_bumper ? 0.5f : 1f), 0f});
        }
        else if(shiftBackward && !shiftBackwardLast) {
            shiftSteps.add(new Float[] {(gamepad1.left_bumper ? -0.5f : -1f), 0f});
        }
        else if(shiftLeft && !shiftLeftLast) {
            shiftSteps.add(new Float[] {0f, (gamepad1.left_bumper ? -0.5f : -1f)});
        }
        else if(shiftRight && !shiftRightLast) {
            shiftSteps.add(new Float[] {0f, (gamepad1.left_bumper ? 0.5f : 1f)});
        }

        if(currentlyShifting) return;

        if(shiftSteps.size() > 0) {
            tileShiftThread = new Thread() {
                public void run() {
                    currentlyShifting = true;

                    Float[] shift = shiftSteps.get(0);
                    shiftSteps.remove(0);

                    if (shift[0] != 0f) {
                        robot.driveTiles(shift[0], driveSpeedMultiplier);
                    } else {
                        robot.strafeTiles(shift[1], driveSpeedMultiplier);
                    }

                    hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    currentlyShifting = false;
                }

                @Override
                public void interrupt() {
                    hardware.getFLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.getFRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.getBLMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.getBRMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    currentlyShifting = false;

                    super.interrupt();
                }
            };
            tileShiftThread.start();
        }

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
    }
    private void manipulator() {
        final float liftSpeed = 1f;
        final float armPosLeft = 0.05f;
        final float armPosCenter = 0.4f;
        final float armPosRight = 0.8f;
        final float clawOpenPos = 0.4f;
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
    }
    private void telemetry() {
        telemetry.addData("Lift Pos", this.hardware.getLift().getCurrentPosition());
        telemetry.addData("Claw Pos", this.hardware.getClaw().getPosition());
        telemetry.addData("Drive Speed Shifter", driveSpeedMultiplier);
        telemetry.update();
    }
    private void afterFrame() {
        decreaseSpeedLast = decreaseSpeed;
        increaseSpeedLast = increaseSpeed;

        changeDriverModeLast = changeDriverMode;

        shiftForwardLast = shiftForward;
        shiftBackwardLast = shiftBackward;
        shiftLeftLast = shiftLeft;
        shiftRightLast = shiftRight;
    }
}
