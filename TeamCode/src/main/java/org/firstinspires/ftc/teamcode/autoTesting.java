package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name="autoTesting")
//@Disabled

public class autoTesting extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotHardware robot = new robotHardware(hardwareMap);
        robot.odo.resetPosAndIMU();

        while (!isStarted() && !isStopRequested())
        {}
        while (true && opModeIsActive()){

            //robot.mecanumDrive(.5, 0, 0, .5);
            //sleep(500);

            //robot.mecanumDrive(0, 0, 0, 0);
            //sleep(2000);

            //robot.mecanumDrive(-.5, 0, 0, .5);
            //sleep(500);

            //robot.mecanumDrive(0, 0, 0, 0);
            //sleep(2000);

            robot.changeSpeed(.25,.25);

            robot.goToPosSingle(20,0,0,0);
            robot.wait(750,robot.odometers);

            telemetry.addData("I'mHere1", robot.odo.getPosX());
            telemetry.update();

            robot.goToPosSingle(0,0,0,Math.toRadians(180));
            robot.wait(750,robot.odometers);

            telemetry.addData("I'mHere1", robot.odo.getPosX());
            telemetry.update();




        }
    }
}
