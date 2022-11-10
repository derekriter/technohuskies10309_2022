package org.firstinspires.ftc.team10309.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;

import org.firstinspires.ftc.team10309.API.Robot;

@Autonomous(name="Dead Wheel Test", group="Examples")
public class DeadWheelTest extends LinearOpMode {
    
    private Robot robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new Robot(this);
        
        waitForStart();
        Thread test = new Thread() {
            @Override
            public synchronized void start() {
                super.start();
            }
            
            DigitalChannelImpl hOdoA = hardwareMap.get(DigitalChannelImpl.class, "hOdoA");
            DigitalChannelImpl hOdoB = hardwareMap.get(DigitalChannelImpl.class, "hOdoB");
//            DigitalChannelImpl vOdoA = hardwareMap.get(DigitalChannelImpl.class, "vOdoA");
//            DigitalChannelImpl vOdoB = hardwareMap.get(DigitalChannelImpl.class, "vOdoB");
            long nanoTimeStart;
            long nanoTimeEnd;
            double ticks = 0;
            @Override
            public void run() {
                hOdoA.setMode(DigitalChannel.Mode.INPUT);
                hOdoB.setMode(DigitalChannel.Mode.INPUT);
                boolean chA = hOdoA.getState();
                boolean chB = hOdoB.getState();
                boolean prevChA = hOdoA.getState();
                boolean prevChB = hOdoB.getState();
//                boolean direction = true;
                while (true) {
                    nanoTimeStart = System.nanoTime();
                    chA = hOdoA.getState();
                    chB = hOdoB .getState();
                    if (prevChA == false && chA == true) {
                        // do something
                        if (chB) {
//                            direction = false;
                            ticks--;
                        } else {
//                            direction = true;
                            ticks++;
                        }
                        prevChA = true;
                    }
                    else if (prevChA == true && chA == false) {
                        if (!chB) {
//                            direction = false;
                            ticks--;
                        } else {
//                            direction = true;
                            ticks++;
                        }
                        prevChA = false;
                    }
//                    telemetry.addData("Ticks", ticks);
//                    telemetry.addData("Direction", direction);
                    
                    
                    nanoTimeEnd = System.nanoTime();
                    telemetry.addData("Speed ",
                            (double) (nanoTimeStart-nanoTimeEnd) / 1_000_000_000);
                    telemetry.update();
                }
            }
    
            @Override
            public void interrupt() {
                super.interrupt();
            }
        };
        test.start();
        while (test.isAlive() && !this.isStopRequested());
        test.interrupt();
        Thread.sleep(1000);
        
    }
}
