package org.firstinspires.ftc.team10309.autoOp.master;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.ManipulatorController;
import org.firstinspires.ftc.team10309.API.Robot;
import org.firstinspires.ftc.team10309.API.SleeveDetect;
import org.firstinspires.ftc.team10309.API.info.RobotInfo;

@Autonomous(name="REDConeV2 | FINAL", group="Examples")
public class REDConeV2 extends LinearOpMode {

    private Robot robot;
    private ManipulatorController manipulatorController;

    @Override
    public void runOpMode() throws InterruptedException {

        final float liftSpeed = 0.9f;
        final float armPosLeft = 0.05f;
        final float armPosCenter = 0.4f;
        final float armPosRight = 0.75f;
        final float clawOpenPos = 0.4f;
        final float clawClosePos = 0.15f;
        //init
        this.robot = new Robot(this, true);
        this.manipulatorController = new ManipulatorController(this.robot.getHardware(), this);

        robot.getHardware().getClaw().setPosition(clawOpenPos);
        Thread.sleep(3000);
        robot.getHardware().getClaw().setPosition(clawClosePos);
        robot.initDetect();
        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();
        SleeveDetect.SignalState state = robot.scanSleeve();
        telemetry.addData("Signal state", state.name());
        telemetry.update();

        // constants, copied from TeleOpMain.

//
//            SleeveDetect.SignalState state = robot.scanSleeve();
//            telemetry.addData("Signal state", state.name());
//            telemetry.update();
//
//            Thread.sleep(1000);
//            robot.turn(-90,.05f,1f); // FIx,  Plus move out before....
//            robot.strafe(21,.7f);
//            robot.drive(32,.7f); // a bit more?
//            robot.drive(2,.7f);
//            clawController.setClawState(ClawController.ClawState.OPEN);
//            robot.strafe(26,.7f);
//            robot.strafeTiles(1, 0.5f, 3f);
//            robot.strafe(-3, 0.5f);
//            robot.driveTiles(1, 0.5f);
//            robot.strafeTiles(1, 0.5f);
//            robot.getHardware().getLift().setTargetPosition(-1100);// Lowest elevator height for
//            // moving claw rotator
//            robot.getHardware().getLift().setPower(liftSpeed);
//            clawController.setClawState(ClawController.ClawState.OPEN);
//            robot.getHardware().getClawRotator().setPosition(armPosLeft);
//
        Thread moveElevator = new Thread() {
            @Override
            public synchronized void start() {
                super.start();
            }

            @Override
            public void run() {
                robot.getHardware().getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.getHardware().getLift().setTargetPosition(RobotInfo.liftTop);
                robot.getHardware().getLift().setPower(-0.8);
            }

            @Override
            public void interrupt() {
                super.interrupt();
            }
        };
        moveElevator.start();
        robot.strafeTiles(.25f, 0.5f);
        robot.driveTiles(-1.2f, 0.5f, 5);
        robot.strafeTiles(1.3f, 0.5f, 0.5f);
        while(robot.getHardware().getLift().isBusy() && opModeIsActive()) {
        }
        robot.drive(-4f, 0.2f);
//          clawController.setClawRotation(ClawController.ClawRotation.BACK);
//            Thread.sleep(1000);
//        Thread.sleep(3000);
//        clawController.setClawRotation(backward);
        Thread.sleep(1000);
        manipulatorController.setClaw(ManipulatorController.ClawPosition.OPEN);
        robot.drive(4.5f, 0.2f);
        robot.strafeTiles(0.4f, 0.4f);
        // LOWER LIFT HERE
        manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND, false);
        manipulatorController.setArmPosition(ManipulatorController.ArmRotation.FRONT);
        manipulatorController.setClaw(ManipulatorController.ClawPosition.CLOSED);
        if (state == SleeveDetect.SignalState.BLUE_TRIANGLE) {
            // stay
            robot.drive(-3, 0.3f);
        } else if (state == SleeveDetect.SignalState.GREEN_CIRCLE) {
            // move back one tile
    
            robot.driveTiles(1, 0.4f, -2);
        } else {
            // move back two tiles
            robot.driveTiles(2, 0.4f, -2);
        }
        while (robot.getHardware().getLift().isBusy() && this.opModeIsActive()) {}
//
//        Thread lowerLift = new Thread() {
//            @Override
//            public synchronized void start() {
//                super.start();
//            }
//
//            @Override
//            public void run() {
//                clawController.setClaw(ClawController.ClawPosition.CLOSED);
//                clawController.setClawRotation(ClawController.ClawRotation.BACK);
//                clawController.setClaw(ClawController.ClawPosition.OPEN);
//                clawController.setLiftPosition(-2390);
//            }
//        };
//        lowerLift.start();
//        robot.driveTiles(-2f, 0.2f);
//        while(lowerLift.isAlive()) {}
//        robot.drive(-4f, 0.2f);
//        clawController.setClaw(ClawController.ClawPosition.CLOSED);
//        clawController.setLiftPosition(-4895);
//        clawController.setClawRotation(ClawController.ClawRotation.FRONT);
//        clawController.setLiftPosition(ClawController.LiftPosition.GROUND);
//
//        if (state == SleeveDetect.SignalState.RED_SQUARE) {
//            robot.driveTiles(2, 0.5f, 3);
//        }
//        else if (state == SleeveDetect.SignalState.GREEN_CIRCLE || state == SleeveDetect.SignalState.ERROR) {
//            robot.driveTiles(1, 0.5f, 3);
//        }
    }
}

