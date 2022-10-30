package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team10309.API.ClawController;
import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="Testing TeleOp")
public class TestingTeleOp extends LinearOpMode {

    private RobotHardware hardware;
    private ClawController clawController;

    @Override
    public void runOpMode() {
        this.hardware = new RobotHardware(this, true);
        this.clawController = new ClawController(this.hardware, this);

        waitForStart();

        while(opModeIsActive()) {
            customLoop();
        }
    }

    public void customLoop() {
        if(gamepad1.a) clawController.setLiftPosition(ClawController.LiftPosition.GROUND);
        else if(gamepad1.x) clawController.setLiftPosition(ClawController.LiftPosition.LOW);
        else if(gamepad1.y) clawController.setLiftPosition(ClawController.LiftPosition.MIDDLE);
        else if(gamepad1.b) clawController.setLiftPosition(ClawController.LiftPosition.HIGH);

        telemetry.addData("Lift Pos", this.hardware.getLift().getCurrentPosition());
        telemetry.addData("Arm Pos", this.hardware.getClawRotater().getPosition());
        telemetry.addData("Claw Pos", this.hardware.getClaw().getPosition());
        telemetry.update();
    }
}
