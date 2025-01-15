package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        robotHardwarePinPoint robot = new robotHardwarePinPoint(hardwareMap);

        robot.resetDriveEncoders();
        robot.odo.resetPosAndIMU();

        intake = hardwareMap.crservo.get("intake");
        extensionWrist = hardwareMap.servo.get("extensionWrist");
        horizontalExtension = hardwareMap.servo.get("horizontalExtension");
        bucketWrist = hardwareMap.servo.get("bucketWrist");
        bucketArm = hardwareMap.servo.get("bucketArm");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);



        while(!isStarted() && !isStopRequested()){
            horizontalExtension.setPosition(.1);
            extensionWrist.setPosition(.5);
            intake.setPower(0);

        }

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(.5,.5);




        /**
         *
         *
         *
         *
         *
         *
         *
         * First sample being dumped
         *
         */

        //double x = -15,y = 7, finalAngle = Math.toRadians(45);
        double x = -10, y = -19, finalAngle = Math.toRadians(0);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2300);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2300);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //double[] fake = robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));
            //telemetry.addData("distanceToTarget", fake[0]);
            //telemetry.addData("absoluteAngleToTarget", Math.toDegrees(fake[1]));
            //telemetry.addData("reletiveXToTarget", fake[2]);
            //telemetry.addData("reletiveYToTarget", fake[3]);
            //telemetry.addData("movementXpower", fake[4]);
            //telemetry.addData("movementYpower", fake[5]);
            //telemetry.addData("movementTurnPower", fake[6]);
            //telemetry.addData("reletiveTurnAngle", Math.toDegrees(fake[7]));
            //telemetry.addData("reletiveAngleToTarget", Math.toDegrees(fake[8]));
            //telemetry.addData("X",robot.GlobalX);
            //telemetry.addData("Y",robot.GlobalY);
            //telemetry.addData("Heading",Math.toDegrees(robot.GlobalHeading));
            //telemetry.update();


        }
        //robot.goToPos(-17,0,Math.toRadians(0),Math.toRadians(180));
        //robot.mecanumDrive(0,0,0,.6);


        robot.changeSpeed(1,1);








    }
}
