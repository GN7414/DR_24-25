package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="BucketSideAuto")
//@Disabled

public class BucketSideAuto extends LinearOpMode
{
    public Servo extensionWrist = null;
    public CRServo intake = null;
    public Servo horizontalExtension = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;

    public DcMotor slidesR = null;
    public DcMotor slidesL = null;

    public static double time;

    public static boolean timerInitted = false;
    public static boolean timerInitted2 = false;
    public static boolean timerInitted3 = false;
    public static ElapsedTime outputTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException
    {

        robotHardwareGen2 robot = new robotHardwareGen2(hardwareMap);

        robot.resetDriveEncoders();
        robot.odo.resetPosAndIMU();



        while(!isStarted() && !isStopRequested()){


        }

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(1,1);

        while(!isStopRequested()){
            double[] fake = robot.goToPosSingle(20, 0, Math.toRadians(90), Math.toRadians(0));
            telemetry.addData("distanceToTarget", fake[0]);
            telemetry.addData("absoluteAngleToTarget", Math.toDegrees(fake[1]));
            telemetry.addData("reletiveXToTarget", fake[2]);
            telemetry.addData("reletiveYToTarget", fake[3]);
            telemetry.addData("movementXpower", fake[4]);
            telemetry.addData("movementYpower", fake[5]);
            telemetry.addData("movementTurnPower", fake[6]);
            telemetry.addData("reletiveTurnAngle", Math.toDegrees(fake[7]));
            telemetry.addData("reletiveAngleToTarget", Math.toDegrees(fake[8]));

            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);
            telemetry.addData("Heading",Math.toDegrees(robot.GlobalHeading));

            telemetry.addData("",null);

            telemetry.addData("turnPowerRight", robot.turnPowerRight);
            telemetry.addData("aTan degrees", robot.aTan);
            telemetry.addData("newAngle", robot.newAngle);
            telemetry.addData("oppo angle ", robot.oppositeAngle);
            telemetry.addData("rotations ", robot.rotations);
            telemetry.addData("robot power", robot.power);
            telemetry.addData("robot turn power", robot.turnPower);
            telemetry.addData("", null);
            telemetry.addData("Powers", null);
            telemetry.addData("right 1", robot.RightOutside.getPower());
            telemetry.addData("right 2", robot.RightInside.getPower());
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


        //robot.goToPos(0, 0, Math.toRadians(90), Math.toRadians(0));


    }







}
