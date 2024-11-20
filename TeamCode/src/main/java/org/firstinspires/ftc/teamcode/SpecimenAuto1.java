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



    public static double x,y, finalAngle;


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
        //armFingerR = hardwareMap.servo.get("frontClawRight");
        //armFingerL = hardwareMap.servo.get("frontClawLeft");
        //slideFingerL = hardwareMap.servo.get("");
        //slideFingerR = hardwareMap.servo.get("");
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


            arm.setTargetPosition(0);
            arm.setPower(0.175);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.changeAccuracy(1,Math.toRadians(1));
            robot.changeSpeed(1,1);

        }


        firstPlace(robot);

        firstGrab(robot);






    }

    public void firstPlace(robotHardware robot){


        robot.changeSpeed(.8,.8);
        x = -6;y = 29.75; finalAngle = Math.toRadians(0);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(-90));


            armWrist.setPosition(robot.WRIST_LOW);
            arm.setTargetPosition(robot.ARM_MID);
            arm.setPower(.25);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slides.setTargetPosition(2100);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        robot.wait(250,robot.odometers);


        armWrist.setPosition(robot.WRIST_TOP);
        armEncoder = robot.ARM_LOW;
        armPower = .25;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.setTargetPosition(1300);
        slides.setPower(1);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(650,robot.odometers);

        specimenGrab.setPosition(.3);//open position

        robot.wait(250,robot.odometers);

    }

    public void firstGrab(robotHardware robot){

        robot.changeAccuracy(3,Math.toRadians(5));
        robot.changeSpeed(.8,.8);
        robot.goToPos(-6, 19, 0, Math.toRadians(0));

        robot.wait(500,robot.odometers);

        x = 20.4;y = 23.6; finalAngle = Math.toRadians(30);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(30));

            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armWrist.setPosition(robot.WRIST_LOW);
            arm.setTargetPosition(robot.ARM_LOW);
            arm.setPower(.25);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            inTake.setPower(.75);

        }
        robot.mecanumDrive(0,0,0,.6);//breaks
        robot.wait(1500,robot.odometers);

        armWrist.setPosition(robot.WRIST_TOP);
        arm.setTargetPosition(robot.ARM_TOP);
        arm.setPower(0.4);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }




}