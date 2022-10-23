package org.firstinspires.ftc.team10309.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="ManipulatorJack", group="Examples")
public class ManipulatorJack extends OpMode {

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

        boolean Raise = this.gamepad2.y;
        boolean Lower = this.gamepad2.a;
        boolean Grab = this.gamepad2.right_bumper;
        boolean Release = this.gamepad2.left_bumper;
        double Liftpos = this.hardware.getLift().getCurrentPosition();

        telemetry.addData("Liftpos", Liftpos);
        telemetry.update();

        if(Raise){
            this.hardware.getLift().setPower(-0.5);
        }

        if(Lower){
            this.hardware.getLift().setPower(0.5);
        }
    }
}
