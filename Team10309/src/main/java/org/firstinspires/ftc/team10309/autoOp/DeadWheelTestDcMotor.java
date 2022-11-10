package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Dead Wheel Test Dc Motor", group="Examples")
public class DeadWheelTestDcMotor extends LinearOpMode {
    
    private Robot robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this);
        
        waitForStart();
        Thread test = new Thread() {
            private DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "front left");
    
            @Override
            public void run() {
                encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (true) {
                    
                    telemetry.addData("Pos", encoder.getCurrentPosition());
                    telemetry.update();
                }
            }
        };
        test.start();
        while (test.isAlive() && !this.isStopRequested());
        test.interrupt();
        Thread.sleep(1000);
    }
}
