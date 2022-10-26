package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.team10309.API.ClawController;
import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="Testing TeleOp")
public class TestingTeleOp extends OpMode {

    private RobotHardware hardware;
    private ClawController clawController;

    @Override
    public void init() {
        this.hardware = new RobotHardware(this, true);
        this.clawController = new ClawController(this.hardware);
    }

    @Override
    public void loop() {
        if(gamepad1.a) clawController.setLiftPosition(ClawController.LiftPosition.GROUND);
        else if(gamepad1.x) clawController.setLiftPosition(ClawController.LiftPosition.LOW);
        else if(gamepad1.y) clawController.setLiftPosition(ClawController.LiftPosition.MIDDLE);
        else if(gamepad1.b) clawController.setLiftPosition(ClawController.LiftPosition.HIGH);

        if(gamepad1.dpad_left) clawController.setClawRotation(ClawController.ClawRotation.LEFT);
        else if(gamepad1.dpad_up) clawController.setClawRotation(ClawController.ClawRotation.MIDDLE);
        else if(gamepad1.dpad_right) clawController.setClawRotation(ClawController.ClawRotation.RIGHT);

        if(gamepad1.left_bumper) clawController.setClawState(ClawController.ClawState.CLOSED);
        else if(gamepad1.right_bumper) clawController.setClawState(ClawController.ClawState.OPEN);
    }
}
