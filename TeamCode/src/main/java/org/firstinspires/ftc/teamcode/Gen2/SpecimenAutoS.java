package org.firstinspires.ftc.teamcode.Gen2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHardware;

@Autonomous(name="SpecimenAutoS")
//@Disabled

public class SpecimenAutoS extends LinearOpMode {
    public DcMotor slidesR = null;
    public DcMotor slidesL = null;
    public DcMotor horizontalExtension = null;
    public DcMotor motorMTConverter = null;

    public Servo extensionWrist = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;
    public Servo turret = null;
    public Servo MTConverter = null;
    public Servo specimenArm = null;
    public Servo specimenWrist = null;
    public Servo specimenClaw = null;

    public CRServo intake = null;

    public double WristPosition = 0;
    public int SlidesPosition = 0;

    public double Position =.5;
    public double HEPosition =.1;

    public double SPEED = .5;

    double[] timeArray = new double[20];

    public static boolean timerInitted = false;
    public static boolean timerInitted2 = false;
    public static boolean timerInitted3 = false;

    private final int TEMP_DOWN = 640;

    public static double x, y, finalAngle;

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardwarePinPoint robot = new robotHardwarePinPoint(hardwareMap);

        robot.resetDriveEncoders();
        robot.odo.resetPosAndIMU();

        intake = hardwareMap.crservo.get("intake");
        extensionWrist = hardwareMap.servo.get("extensionWrist");

        bucketWrist = hardwareMap.servo.get("bucketWrist");
        bucketArm = hardwareMap.servo.get("bucketArm");
        turret = hardwareMap.servo.get("turret");
        MTConverter = hardwareMap.servo.get("MTConverter");
        specimenArm = hardwareMap.servo.get("specimenArm");
        specimenClaw = hardwareMap.servo.get("specimenClaw");
        specimenWrist = hardwareMap.servo.get("specimenWrist");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");
        motorMTConverter = hardwareMap.dcMotor.get("motorMTConverter");
        horizontalExtension = hardwareMap.dcMotor.get("horizontalExtension");

        horizontalExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMTConverter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMTConverter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //intake.setPower(Position);...........


        while (!isStarted() && !isStopRequested()) {
            extensionWrist.setPosition(0);  //Tuned
            turret.setPosition(robot.TURRET_LEFT); //Tuned
            intake.setPower(0);  //Tuned
            bucketWrist.setPosition(1);  //Tuned
            bucketArm.setPosition(.99);  //Tuned
            //MTConverter.setPosition(1);  //Tuned
            specimenArm.setPosition(.1);
            specimenClaw.setPosition(1);
            specimenWrist.setPosition(.65);

            SlidesPosition = (int) robot.SLIDE_INIT;
            slidesL.setTargetPosition(SlidesPosition);
            slidesL.setPower(.2);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesR.setTargetPosition(SlidesPosition);
            slidesR.setPower(.2);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            horizontalExtension.setTargetPosition(0);
            horizontalExtension.setPower(.5);
            horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);  //Tuned

            //motorMTConverter.setTargetPosition(0);
            //motorMTConverter.setPower(.2);
            //motorMTConverter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.changeAccuracy(1, Math.toRadians(1));
            robot.changeSpeed(1, 1);

        }


        firstPlace(robot);

        //firstPickup(robot);

        //secondPickup(robot);

        //thirdPickup(robot);

        firstCycle(robot);


    }

    public void firstPlace(robotHardwarePinPoint robot) {

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(.8,.8);

        x = 30.75;
        y = 12;
        finalAngle = Math.toRadians(0);

        slidesR.setTargetPosition(1500);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(1500);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specimenArm.setPosition(robot.SPECIMEN_ARM_PLACE);
        specimenWrist.setPosition(robot.SPECIMEN_WRIST_PLACE);

        robot.wait(400,robot.odometers);

        time = robot.currentTime.milliseconds() + 5000;
        while((Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) && !robot.boolTimer(time)){

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));

        }


        robot.mecanumDrive(0,0,0,0);

        robot.changeAccuracy(2,Math.toRadians(5));
        robot.changeSpeed(1,1);

        x = 18;
        y = 12;
        finalAngle = Math.toRadians(0);
        robot.goToPos(x,y,finalAngle,Math.toRadians(180));

        robot.wait(600,robot.odometers);

        specimenArm.setPosition(robot.SPECIMEN_ARM_PICKUP);
        specimenWrist.setPosition(robot.SPECIMEN_WRIST_PICKUP);

    }

    public void firstPickup(robotHardwarePinPoint robot) {

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(0.75,0.75);

        x = 22;
        y = -8;
        finalAngle = Math.toRadians(-60);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));


            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        turret.setPosition(robot.TURRET_RIGHT);
        horizontalExtension.setTargetPosition(975);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500,robot.odometers);

        intake.setPower(-1);
        horizontalExtension.setTargetPosition(500);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(250,robot.odometers);

        //robot.goToPos(22,-9,Math.toRadians(-30),Math.toRadians(0));

        //robot.wait(250,robot.odometers);

        robot.goToPos(22,-9,Math.toRadians(-90),Math.toRadians(0));

        robot.wait(250,robot.odometers);

        intake.setPower(1);

        robot.wait(250,robot.odometers);

    }

    public void secondPickup(robotHardwarePinPoint robot) {

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(0.75,0.75);

        x = 24;
        y = -21;
        finalAngle = Math.toRadians(-70);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));


            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        turret.setPosition(robot.TURRET_LEFT);
        horizontalExtension.setTargetPosition(700);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500,robot.odometers);

        intake.setPower(-1);
        horizontalExtension.setTargetPosition(400);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(250,robot.odometers);

        robot.goToPos(24,-22,Math.toRadians(-70),Math.toRadians(0));

        robot.wait(250,robot.odometers);

        robot.goToPos(24,-22,Math.toRadians(0),Math.toRadians(-70));

        robot.wait(250,robot.odometers);

        intake.setPower(1);

        robot.wait(250,robot.odometers);



    }

    public void thirdPickup(robotHardwarePinPoint robot) {

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(0.75,0.75);

        x = 23;
        y = -29;
        finalAngle = Math.toRadians(-70);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));


            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.mecanumDrive(0,0,0,0);

        extensionWrist.setPosition(robot.WRIST_PICKUP);
        turret.setPosition(robot.TURRET_LEFT);
        horizontalExtension.setTargetPosition(500);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(500,robot.odometers);

        intake.setPower(-1);
        horizontalExtension.setTargetPosition(200);
        horizontalExtension.setPower(1);
        horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(250,robot.odometers);

        robot.goToPos(23,-29,Math.toRadians(-70),Math.toRadians(0));

        robot.wait(250,robot.odometers);

        robot.goToPos(23,-29,Math.toRadians(0),Math.toRadians(-70));

        robot.wait(250,robot.odometers);

        intake.setPower(1);

        robot.wait(250,robot.odometers);




    }

    public void firstCycle(robotHardwarePinPoint robot) {

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(0.75,0.75);

        //time = robot.currentTime.milliseconds() + 6000;

        x = 0;
        y = -23;
        finalAngle = Math.toRadians(0);

        while((Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy)) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));

            slidesR.setTargetPosition(100);
            slidesR.setPower(1);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesL.setTargetPosition(100);
            slidesL.setPower(1);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            specimenArm.setPosition(robot.SPECIMEN_ARM_PICKUP);
            specimenWrist.setPosition(robot.SPECIMEN_WRIST_PICKUP);

        }

        specimenClaw.setPosition(1);

        robot.wait(500,robot.odometers);

        slidesR.setTargetPosition(1500);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(1500);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specimenArm.setPosition(robot.SPECIMEN_ARM_PLACE);
        specimenWrist.setPosition(robot.SPECIMEN_WRIST_PLACE);

        robot.mecanumDrive(0,0,0,0);

        robot.wait(250,robot.odometers);

        x = 31;
        y = 12;
        finalAngle = Math.toRadians(0);
        robot.goToPos(x,y,finalAngle,0);

        robot.changeAccuracy(2,Math.toRadians(5));

        x = 26;
        finalAngle = Math.toRadians(0);
        robot.goToPos(x,y,finalAngle,0);

        robot.changeAccuracy(1,Math.toRadians(1));


    }

    public void park(robotHardwarePinPoint robot) {

        robot.changeAccuracy(5, Math.toRadians(10));

        slidesR.setTargetPosition(0);
        slidesR.setPower(1);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setTargetPosition(0);
        slidesL.setPower(1);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.goToPos(10,0,Math.toRadians(0), Math.toRadians(0));

        robot.goToPos(5,30,Math.toRadians(0), Math.toRadians(0));


    }

}