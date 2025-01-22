package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="swerveAttempt2_0")
@Disabled

public class swerveAttempt2_0 extends LinearOpMode
{


    @Override
    public void runOpMode() throws InterruptedException {

        swerveRobotHardware robot = new swerveRobotHardware(hardwareMap);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();


        waitForStart();

        while (opModeIsActive()) {

            robot.swerveDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);



            telemetry.addData("turnPowerRight", robot.turnPowerRight);
            telemetry.addData("aTan degrees", robot.aTan);
            telemetry.addData("newAngle", robot.newAngle);
            telemetry.addData("oppo angle ", robot.oppositeAngle);
            telemetry.addData("rotations ", robot.rotations);
            telemetry.addData("robot power", robot.power);
            telemetry.addData("", null);
            telemetry.addData("Powers", null);
            telemetry.addData("right 1", robot.RightOutside.getPower());
            telemetry.addData("right 2", robot.RightInside.getPower());
            telemetry.addData("Location:", null);
            telemetry.addData("left 1", robot.LeftOutside.getPower());
            telemetry.addData("left 2", robot.LeftInside.getPower());

            //one +     two -
            telemetry.addData("right pod pos", robot.rightPodPosition);
            telemetry.addData("left pod pos", robot.leftPodPosition);
            telemetry.addData("current angle right",robot.currentAngle);
            telemetry.addData("final angle",robot.finalAngle);
            telemetry.addData("wheel Direction",robot.wheelDirection);
            telemetry.addData("distance",robot.distance);
            telemetry.addData("oppo-distance",robot.oppositeDistance);
            telemetry.addData("testing",robot.testing);
            telemetry.update();

        }
    }
}
