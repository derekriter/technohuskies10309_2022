package org.firstinspires.ftc.team10309.autoOp.master;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team10309.API.ClawController;
import org.firstinspires.ftc.team10309.API.Robot;
import org.firstinspires.ftc.team10309.API.SleeveDetect;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Autonomous(name="SbTb | FINAL Auto Op")
public class SbTbAutoOP extends LinearOpMode {

    private Robot robot;
    private ClawController clawController;

    @Override
    public void runOpMode() throws InterruptedException {
        //consts
        final float speed = 0.7f;

        //init
        this.robot = new Robot(this, false);
        this.clawController = new ClawController(this.robot.getHardware());

        this.clawController.setClawRotation(ClawController.ClawRotation.BACK);
        this.clawController.setLiftPosition(ClawController.LiftPosition.GROUND);
        this.clawController.setClawState(ClawController.ClawState.OPEN);

        //wait for 5 seconds for the driver to insert the preloaded cone
        wait(5000);

        this.clawController.setClawState(ClawController.ClawState.CLOSED);

        SleeveDetect.SignalState sleeveState = this.robot.initAndScan();
        telemetry.addData("Sleeve State", sleeveState.name());
        telemetry.update();

        waitForStart();

        robot.driveTiles(1,speed);
        robot.strafeTiles(2, speed);
        robot.driveTiles(-0.5f, speed);

        List<Double> stackTimes = new ArrayList<>();
        double biggest = 0;

        while(getRuntime() + biggest + 10 <= 30) {
            double stackStartTime = getRuntime();

            //stack cone
            this.clawController.setLiftPosition(ClawController.LiftPosition.HIGH);
            this.clawController.setClawRotation(ClawController.ClawRotation.SIDE);
            this.clawController.setClawState(ClawController.ClawState.OPEN);

            //move manipulator to position for grabbing cone
            this.clawController.setClawRotation(ClawController.ClawRotation.BACK);
            this.clawController.setLiftPosition(ClawController.LiftPosition.GROUND);

            //grab cone
            this.robot.driveTiles(-1.2f, speed);
            this.clawController.setClawState(ClawController.ClawState.CLOSED);
            this.robot.driveTiles(1.2f, speed);

            stackTimes.add(getRuntime() - stackStartTime);
            biggest = stackTimes.stream().max(Comparator.naturalOrder()).get();
        }
        this.clawController.setLiftPosition(ClawController.LiftPosition.HIGH);
        this.clawController.setClawRotation(ClawController.ClawRotation.SIDE);
        this.clawController.setClawState(ClawController.ClawState.OPEN);

        this.clawController.setClawRotation(ClawController.ClawRotation.FRONT);
        this.clawController.setLiftPosition(ClawController.LiftPosition.GROUND);

        //park in area indicated by the signal
        if(sleeveState == SleeveDetect.SignalState.RED_SQUARE) {
            this.robot.driveTiles(0.5f, speed);
        }
        else if(sleeveState == SleeveDetect.SignalState.GREEN_CIRCLE || sleeveState == SleeveDetect.SignalState.ERROR) {
            this.robot.driveTiles(-0.5f, speed);
        }
        else {
            this.robot.driveTiles(-1.5f, speed);
        }
    }
}