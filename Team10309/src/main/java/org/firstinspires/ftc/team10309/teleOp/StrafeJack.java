//package org.firstinspires.ftc.team10309.teleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.team10309.API.RobotHardware;
//
//@TeleOp(name="StrafeJack")
//public class StrafeJack extends LinearOpMode {
//
//    public RobotHardware hardware;
//
//    @Override
//    public void runOpMode() {
//        this.hardware = new RobotHardware(this, true);
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            customLoop();
//        }
//    }
//
//    public void customLoop() {
//
//        double strafe = this.gamepad1.left_stick_x;
//        double forward = -this.gamepad1.left_stick_y;
//        double turn = this.gamepad1.right_stick_x;
//
//        double FLSpeed = forward + strafe + turn;
//        double FRSpeed = forward - strafe - turn;
//        double BRSpeed = forward + strafe - turn;
//        double BLSpeed = forward - strafe + turn;
//
//        double largest = 1.0;
//        largest=Math.max(largest, Math.abs(FLSpeed));
//        largest=Math.max(largest, Math.abs(FRSpeed));
//        largest=Math.max(largest, Math.abs(BLSpeed));
//        largest=Math.max(largest, Math.abs(BRSpeed));
//
//        this.hardware.getFLMotor().setPower(FLSpeed);
//        this.hardware.getBRMotor().setPower(BRSpeed);
//        this.hardware.getFRMotor().setPower(FRSpeed);
//        this.hardware.getBLMotor().setPower(BLSpeed);
//
//        telemetry.addData("Strafe", strafe);
//        telemetry.addData("forward", forward);
//        telemetry.addData("turn", turn);
//        telemetry.update();
//    }
//}
