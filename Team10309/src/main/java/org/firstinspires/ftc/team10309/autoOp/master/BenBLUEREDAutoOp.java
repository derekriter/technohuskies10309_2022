package org.firstinspires.ftc.team10309.autoOp.master;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.ManipulatorController;
import org.firstinspires.ftc.team10309.API.Robot;
import org.firstinspires.ftc.team10309.API.SleeveDetect;

// Algin with a jenga block 1 jenga block from the right side edge of mat.
@Autonomous(name="BenBlueRed")
public class BenBLUEREDAutoOp extends LinearOpMode {

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

        manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.HIGH, false);
        this.robot.driveTiles(2.0f,0.5f, -1f);
        this.robot.turn(45f, 0.2f);
        this.robot.turn(45f, 0.2f);
        this.robot.drive(3f, 0.5f);
        manipulatorController.setArmPosition(ManipulatorController.ArmRotation.FRONT);
        manipulatorController.setClaw(ManipulatorController.ClawPosition.OPEN);
        manipulatorController.setArmPosition(ManipulatorController.ArmRotation.SIDE);
        manipulatorController.setLiftPosition(-1256);
        this.robot.strafeTiles(2f, 0.5f, -3f);
        manipulatorController.setClaw(ManipulatorController.ClawPosition.CLOSED);
        manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.LOW);
        manipulatorController.setArmPosition(ManipulatorController.ArmRotation.FRONT);

        if (state == SleeveDetect.SignalState.RED_SQUARE) {
            this.robot.strafeTiles(-1.5f, 0.5f);
            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);
        } else if (state == SleeveDetect.SignalState.GREEN_CIRCLE) {
            this.robot.strafeTiles(-0.5f, 0.5f);
            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);
        } else if (state == SleeveDetect.SignalState.BLUE_TRIANGLE) {

            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);
        } else {
            this.robot.strafeTiles(-0.5f, 0.5f);
            manipulatorController.setLiftPosition(ManipulatorController.LiftPosition.GROUND);

        }








    }
}