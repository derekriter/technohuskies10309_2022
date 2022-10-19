package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="PowerPlay TeleOp")
public class PowerPlayTeleOp extends OpMode {

    private RobotHardware hardware;

    @Override
    public void init() {
        this.hardware = new RobotHardware(this);
    }

    @Override
    public void loop() {
        double drive = -this.gamepad1.left_stick_y;
        double strafe = this.gamepad1.left_stick_x;
        double turn = this.gamepad1.right_stick_x;

        double FLSpeed = drive + strafe + turn;
        double FRSpeed = drive - strafe - turn;
        double BRSpeed = drive + strafe - turn;
        double BLSpeed = drive - strafe + turn;

        this.hardware.getFLMotor().setPower(FLSpeed);
        this.hardware.getBRMotor().setPower(BRSpeed);
        this.hardware.getFRMotor().setPower(FRSpeed);
        this.hardware.getBLMotor().setPower(BLSpeed);
    }
}
