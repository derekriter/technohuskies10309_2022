package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="StrafeJack")
public class StrafeJack extends OpMode {

    private RobotHardware hardware;
    double left;
    double right;
    double drive;
    double turn;
    double max;


    @Override
    public void init() {
        this.hardware = new RobotHardware(this, false);
    }

    @Override
    public void loop() {

/*        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }*/


        double strafe = this.gamepad1.left_stick_x;
        double forward = -this.gamepad1.left_stick_y;
        double turn = this.gamepad1.right_stick_x;
        
        double FLSpeed = forward + strafe + turn;
        double FRSpeed = forward - strafe - turn;
        double BRSpeed = forward + strafe - turn;
        double BLSpeed = forward - strafe + turn;

        this.hardware.getFLMotor().setPower(FLSpeed);
        this.hardware.getBRMotor().setPower(BRSpeed);
        this.hardware.getFRMotor().setPower(FRSpeed);
        this.hardware.getBLMotor().setPower(BLSpeed);

        // move forward
        // this.hardware.getFLMotor().setPower(-lsy);
//        this.hardware.getFRMotor().setPower(-lsy);
//        this.hardware.getBLMotor().setPower(-lsy);
//        this.hardware.getBRMotor().setPower(-lsy);

        // Turning code
//        this.hardware.getFLMotor().setPower(-rsx);
//        this.hardware.getFRMotor().setPower(rsx);
//        this.hardware.getBRMotor().setPower(rsx);
//        this.hardware.getBLMotor().setPower(-rsx);


        telemetry.addData("Strafe", strafe);
        telemetry.addData("forward", forward);
        telemetry.addData("turn", turn);
        telemetry.update();
    }
}
