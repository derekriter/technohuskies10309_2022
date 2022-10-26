package org.firstinspires.ftc.team10309.teleOp;

import static java.lang.System.in;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team10309.API.RobotHardware;

@TeleOp(name="ManipulatorJack", group="Examples")
public class ManipulatorJack extends OpMode {

    private RobotHardware hardware;

    @Override
    public void init() {
        this.hardware = new RobotHardware(this, true); //MEANT TO BE TRUE!!!
        this.hardware.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void loop() {

        boolean Ground = gamepad2.a;
        boolean Small = gamepad2.b;
        boolean Middle = gamepad2.x;
        boolean High = gamepad2.y;
        double Liftpos = this.hardware.getLift().getCurrentPosition();

        telemetry.addData("Liftpos", Liftpos);
        telemetry.update();

        if(Ground){
            this.hardware.getLift().setTargetPosition(0);   //0 inches
        }

        if(Small){
            this.hardware.getLift().setTargetPosition(3846);    // 10 inches
        }

        if(Middle){
            this.hardware.getLift().setTargetPosition(7692);    //20 inches
        }

        if(High){
            this.hardware.getLift().setTargetPosition(11538);   //30 inches
        }
    }
}
