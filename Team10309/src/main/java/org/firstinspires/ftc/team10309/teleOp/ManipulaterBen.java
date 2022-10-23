package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="manipulater | Ben")
public class ManipulaterBen extends OpMode {

    private RobotHardware hardware;
    double left;
    double right;
    double drive;
    double turn;
    double max;
    @Override
    public void init() {
        this.hardware = new RobotHardware(this, false);
        this.hardware.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        boolean raise = gamepad2.y;
        boolean lower = gamepad2.a;
        boolean grab = gamepad2.right_bumper;
        boolean release = gamepad2.left_bumper;
        double liftPos =  this.hardware.getLift().getCurrentPosition();
        telemetry.addData("liftpos", liftPos);
        telemetry.update();

        if(raise) {
            this.hardware.getLift().setPower(-0.5);
        }
        if(lower) {
            this.hardware.getLift().setPower(0.5);
        }
    }
}

