package org.firstinspires.ftc.team10309.teleOp;

import static java.lang.System.in;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="ManipulatorJack", group="Examples")
public class ManipulatorJack extends LinearOpMode {

    private RobotHardware hardware;

    @Override
    public void runOpMode() {
        this.hardware = new RobotHardware(this, true); //MEANT TO BE TRUE!!!
        this.hardware.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            customLoop();
        }
    }

    public void customLoop() {


        boolean Raise = this.gamepad2.y;
        boolean Lower = this.gamepad2.a;
        double Liftpos = this.hardware.getLift().getCurrentPosition();

        if(Raise){
            this.hardware.getLift().setPower(-0.45);
        }

        if(Lower){
            this.hardware.getLift().setPower(0.45);
        }

        telemetry.addData("Liftpos", Liftpos);
        telemetry.update();
        }
    }

