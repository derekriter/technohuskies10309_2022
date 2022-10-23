package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="Testing TeleOp")
public class TestingTeleOp extends OpMode {

    private RobotHardware hardware;

    @Override
    public void init() {
        this.hardware = new RobotHardware(this, false);
    }

    @Override
    public void loop() {
        this.hardware.getFLMotor().setPower(this.gamepad1.y ? 0.25 : 0);
        this.hardware.getFRMotor().setPower(this.gamepad1.b ? 0.25 : 0);
        this.hardware.getBLMotor().setPower(this.gamepad1.x ? 0.25 : 0);
        this.hardware.getBRMotor().setPower(this.gamepad1.a ? 0.25 : 0);

        telemetry.addData("Left Stick Y", this.gamepad1.left_stick_y);
        telemetry.update();
    }
}
