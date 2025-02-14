package org.firstinspires.ftc.teamcode.Gen2.StateAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gen2.robotHardwarePinPoint;

@Autonomous(name="BucketSideAuto")
public class BucketsideAuto extends LinearOpMode {

    public Servo extensionWrist = null;

    public CRServo intake = null;
    public Servo horizontalExtension = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;
    public Servo turret = null;
    public DcMotor slidesR = null;
    public DcMotor slidesL = null;

    public static double time;

    public static boolean timerInitted = false;

    public static ElapsedTime outputTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotHardwarePinPoint robot = new robotHardwarePinPoint(hardwareMap);

        robot.odo.resetPosAndIMU();
        robot.resetDriveEncoders();

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

        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);

        while(!isStarted() && !isStopRequested()){
            horizontalExtension.setPosition(.1);
            extensionWrist.setPosition(.35);
            intake.setPower(0);

        }


        preloadedSamplePlace(robot);

        firstSamplePickup(robot);

        firstSamplePlace(robot);


    }
    public void preloadedSamplePlace(robotHardwarePinPoint robot){

        robot.changeAccuracy(2,4);
        robot.changeSpeed(0.75,0.75);

        double x = 0;
        double y = 0;
        double finalAngle = Math.toRadians(0);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));


            slidesR.setTargetPosition(2200);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2200);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(850, robot.odometers);

    }
    public void firstSamplePickup(robotHardwarePinPoint robot){

        robot.changeAccuracy(0,0);
        robot.changeSpeed(0.75,0.75);

        bucketArm.setPosition(robot.BUCKET_ARM_REST);
        bucketWrist.setPosition(robot.BUCKET_WRIST_REST);

        slidesR.setTargetPosition(100);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(100);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500, robot.odometers);

        horizontalExtension.setPosition(0);
        extensionWrist.setPosition(robot.WRIST_PICKUP);
        turret.setPosition(robot.TURRET_LEFT);
        intake.setPower(-1);

        robot.wait(750,robot.odometers);

        turret.setPosition(robot.TURRET_MIDDLE);

        robot.wait(150,robot.odometers);

        horizontalExtension.setPosition(0);

        robot.wait(200,robot.odometers);

        extensionWrist.setPosition(robot.WRIST_DROP);

        robot.wait(300,robot.odometers);

        intake.setPower(-1);

        robot.wait(500,robot.odometers);
    }
    public void firstSamplePlace(robotHardwarePinPoint robot){

        robot.changeAccuracy(2,4);
        robot.changeSpeed(0.75,0.75);

        double x = 0;
        double y = 0;
        double finalAngle = Math.toRadians(0);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));


            slidesR.setTargetPosition(2200);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(2200);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
        bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

        robot.wait(850, robot.odometers);

    }
    public void secondSamplePickup(robotHardwarePinPoint robot){







    }


}
