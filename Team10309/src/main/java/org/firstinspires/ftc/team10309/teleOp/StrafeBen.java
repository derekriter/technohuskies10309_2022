package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="Strafe | Ben")
public class StrafeBen extends LinearOpMode {

    private RobotHardware hardware;
    double left;
    double right;
    double drive;
    double turn;
    double max;

    @Override
    public void runOpMode() {
        this.hardware = new RobotHardware(this, false);

        waitForStart();

        while(opModeIsActive()) {
            customLoop();
        }
    }

    public void customLoop() {


//        drive = -gamepad1.left_stick_y;
//        turn  =  gamepad1.right_stick_x;
//
//        // Combine drive and turn for blended motion.
//        left  = drive + turn;
//        right = drive - turn;
//
//        // Normalize the values so neither exceed +/- 1.0
//        max = Math.max(Math.abs(left), Math.abs(right));
//        if (max > 1.0)
//        {
//            left /= max;
//            right /= max;
//        }
        this.hardware.getFLMotor().setPower(gamepad1.right_stick_x);
        this.hardware.getBRMotor().setPower(gamepad1.right_stick_x);
        this.hardware.getFRMotor().setPower(-gamepad1.right_stick_x);
        this.hardware.getBLMotor().setPower(-gamepad1.right_stick_x);
    }
}
