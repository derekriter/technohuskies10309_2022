    package org.firstinspires.ftc.team10309.autoOp;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    import org.firstinspires.ftc.team10309.API.Robot;

    @Autonomous(name="RussellAutoOp", group="Examples")
    public class RussellAutoOp extends LinearOpMode {

        private Robot robot;

        @Override
        public void runOpMode() throws InterruptedException {
            //init
            this.robot = new Robot(this);

            waitForStart();


           //strafe right 2 inches
            robot.strafe(2,0.7f);
            robot.drive(23,0.8f);
            robot.strafe(-26,0.7f);
            robot.drive(25,0.8f);
            robot.strafe(21,0.7f);
            //open claw
            robot.strafe(2,0.7f);
            //close claw
            robot.strafe(22,0.7f);
            //move arm up 34 inches
            //open claw












        }
    }
