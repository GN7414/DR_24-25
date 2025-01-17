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
        robot.changeSpeed(.75,.75);


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


        robot.goToPos(0, 5,0,Math.toRadians(90));

        //double x = -15,y = 7, finalAngle = Math.toRadians(45);
        double x = -19;
        double y = 4;
        double finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2050);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2050);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(750, robot.odometers);

        /**
         *
         * End of first placement
         *
         *
         * Start of going to the second sample
         *
         */


        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);

        x = 5;
        y = 35;
        finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));


            slidesR.setTargetPosition(0);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(0);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        robot.mecanumDrive(0,0,0,0);
        robot.wait(250, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_LOW);
        intake.setPower(-1);

        robot.wait(350, robot.odometers);

        horizontalExtension.setPosition(.2);

        robot.wait(750, robot.odometers);

        horizontalExtension.setPosition(.1);

        robot.wait(750, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_TOP);

        robot.wait(500, robot.odometers);

        intake.setPower(1);

        robot.wait(1000, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_LOW);
        intake.setPower(0);

        robot.wait(250, robot.odometers);

        x = -19;
        y = 4;
        finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2050);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2050);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(750, robot.odometers);

        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);

        extensionWrist.setPosition(.5);

        x = -4;
        y = 35;
        finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));


            slidesR.setTargetPosition(0);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(0);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);
        robot.wait(250, robot.odometers);


        /**
         *
         *
         * prep for third sample
         *
         *
         */

        extensionWrist.setPosition(robot.WRIST_LOW);
        intake.setPower(-1);

        robot.wait(350, robot.odometers);

        horizontalExtension.setPosition(.2);

        robot.wait(750, robot.odometers);

        horizontalExtension.setPosition(.1);

        robot.wait(750, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_TOP);

        robot.wait(500, robot.odometers);

        intake.setPower(1);

        robot.wait(1000, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_LOW);
        intake.setPower(0);

        robot.wait(250, robot.odometers);

        x = -19;
        y = 4;
        finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2050);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2050);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(750, robot.odometers);

        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);


        /**
         *
         *
         * prep for fourth sample
         *
         *
         */

        extensionWrist.setPosition(.5);

        x = -13;
        y = 35;
        finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));


            slidesR.setTargetPosition(0);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(0);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        robot.mecanumDrive(0,0,0,0);
        robot.wait(250, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_LOW);
        intake.setPower(-1);

        robot.wait(350, robot.odometers);

        horizontalExtension.setPosition(.2);

        robot.wait(750, robot.odometers);

        horizontalExtension.setPosition(.1);

        robot.wait(750, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_TOP);

        robot.wait(500, robot.odometers);

        intake.setPower(1);

        robot.wait(1000, robot.odometers);

        extensionWrist.setPosition(robot.WRIST_LOW);
        intake.setPower(0);

        robot.wait(250, robot.odometers);

        x = -19;
        y = 4;
        finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slidesR.setTargetPosition(2050);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2050);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(750, robot.odometers);

        bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest

        robot.wait(500, robot.odometers);

        slidesR.setTargetPosition(0);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(0);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(750, robot.odometers);





    }
}
