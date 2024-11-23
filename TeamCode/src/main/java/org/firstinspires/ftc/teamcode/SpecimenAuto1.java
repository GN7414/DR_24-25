package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="SpecimenAuto1")
//@Disabled

public class SpecimenAuto1 extends LinearOpMode {
    public Servo armWrist = null;
    public Servo dumper = null;
    public CRServo inTake = null;
    public Servo specimenGrab = null;

    public DcMotor arm = null;
    public DcMotor slides = null;
    public int slideEncoder = 0;

    public int downPos = 660;
    public int armEncoder = 0;
    public static double time;
    public double armPower = 0;

    public static boolean timerInitted = false;
    public static boolean timerInitted2 = false;
    public static boolean timerInitted3 = false;
    public static ElapsedTime outputTime = new ElapsedTime();

    private final int TEMP_DOWN = 640;

    public static double x, y, finalAngle;


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();
        robot.odo.resetPosAndIMU();

        //drive motors
        arm = hardwareMap.dcMotor.get("frontArmMotor");
        slides = hardwareMap.dcMotor.get("slides");


        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armWrist = hardwareMap.servo.get("intakeWrist");
        inTake = hardwareMap.crservo.get("intake");
        dumper = hardwareMap.servo.get("bucket");
        specimenGrab = hardwareMap.servo.get("specimenGrab");

        arm.setDirection(DcMotor.Direction.REVERSE);

        /**   INIT
         *
         *
         *
         */

        while (!isStarted() && !isStopRequested()) {

            dumper.setPosition(.4);
            //armWrist.setPosition(.5);
            inTake.setPower(0);
            specimenGrab.setPosition(0);

            //arm.setTargetPosition(50);
            //arm.setPower(0.175);
            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.changeAccuracy(1, Math.toRadians(1));
            robot.changeSpeed(1, 1);

        }


        firstPlace(robot);

        firstCycle(robot);

        place(robot);


    }

    public void firstPlace(robotHardware robot) {


        robot.changeAccuracy(.5, Math.toRadians(1));
        robot.changeSpeed(1, 1);

        x = -6;
        y = 29.75;
        finalAngle = Math.toRadians(0);

        while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));


            armWrist.setPosition(robot.WRIST_LOW);
            arm.setTargetPosition(200);
            arm.setPower(.25);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slides.setTargetPosition(2100);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.wait(250, robot.odometers);

        slides.setTargetPosition(1300);
        slides.setPower(1);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.goToPos(-8, 29.75, Math.toRadians(0), Math.toRadians(-90));

        robot.wait(500, robot.odometers);

        specimenGrab.setPosition(.3);//open position

        armWrist.setPosition(robot.WRIST_LOW);
        arm.setTargetPosition(robot.ARM_MID);
        arm.setPower(.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(250, robot.odometers);

    }

    public void firstCycle(robotHardware robot) {


        robot.changeAccuracy(4, Math.toRadians(7));
        robot.changeSpeed(.8, .8);


        robot.goToPos(-6, 20, Math.toRadians(0), Math.toRadians(-90));

        robot.changeAccuracy(.5, Math.toRadians(1));

        x = 18.4;
        y = 23;
        finalAngle = Math.toRadians(30);

        while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, Math.toRadians(30), Math.toRadians(0));

            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armWrist.setPosition(robot.WRIST_LOW);
            arm.setTargetPosition(robot.ARM_LOW);
            arm.setPower(.25);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            inTake.setPower(.75);

        }
        robot.mecanumDrive(0, 0, 0, .6);//brakes

        arm.setPower(0);


        //move to collect
        time = robot.timerInit(500);
        while (!robot.boolTimer(time)) {

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-.3, 0, 0, 1);

        }
        robotHardware.timerInitted = false;
        robot.mecanumDrive(0, 0, 0, .6);//brakes

        robot.wait(400, robot.odometers);

        robot.goToPos(36.3, 14.6, Math.toRadians(315), Math.toRadians(0));

        inTake.setPower(-.75);

        robot.wait(350, robot.odometers);

        armWrist.setPosition(robot.WRIST_MID);
        arm.setTargetPosition(200);
        arm.setPower(.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.changeAccuracy(1, Math.toRadians(10));

        robot.goToPos(28.7, 14, Math.toRadians(180), Math.toRadians(0));

        robot.changeAccuracy(.25, Math.toRadians(1));
        robot.goToPos(28.7, 0.25, Math.toRadians(180), Math.toRadians(90));

        specimenGrab.setPosition(0);

        robot.wait(250, robot.odometers);

        slides.setTargetPosition(1300);
        slides.setPower(1);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(400, robot.odometers);


    }

    public void secondCycle(robotHardware robot) {


    }

    public void place(robotHardware robot) {

        robot.changeAccuracy(.5, Math.toRadians(3));

        robot.goToPos(5,10,0,Math.toRadians(90));

        robot.changeSpeed(1, 1);

        x = -6;
        y = 30;
        finalAngle = Math.toRadians(0);

        while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(90));


            armWrist.setPosition(robot.WRIST_LOW);
            arm.setTargetPosition(200);
            arm.setPower(.25);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slides.setTargetPosition(2100);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.wait(250, robot.odometers);

        slides.setTargetPosition(1300);
        slides.setPower(1);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.goToPos(-8, 30, Math.toRadians(0), Math.toRadians(-90));

        robot.wait(500, robot.odometers);

        specimenGrab.setPosition(.3);//open position

        armWrist.setPosition(robot.WRIST_LOW);
        arm.setTargetPosition(robot.ARM_MID);
        arm.setPower(.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(350, robot.odometers);

    }
}
