package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoRI30H")
//@Disabled

public class AutoRI30H extends LinearOpMode
{
    public Servo armWrist = null;
    public Servo dumper = null;
    public CRServo inTake = null;

    public DcMotor arm = null;
    public DcMotor slides = null;

    public int downPos = 650;
    public static double time;

    public static boolean timerInitted = false;
    public static boolean timerInitted2 = false;
    public static boolean timerInitted3 = false;
    public static ElapsedTime outputTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException
    {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        //dive motors
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

        //slides.setDirection(DcMotor.Direction.REVERSE);



        while(!isStarted() && !isStopRequested()){

            dumper.setPosition(.4);
            //armWrist.setPosition(.5);
            inTake.setPower(0);


            arm.setTargetPosition(0);
            arm.setPower(0.175);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }


        armWrist.setPosition(.4);
        arm.setTargetPosition(450);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(.8,.8);

        double x = 17,y = -4, finalAngle = Math.toRadians(-45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);


            slides.setTargetPosition(4500);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /*
            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);
            telemetry.addData("heading",robot.GlobalHeading);
            telemetry.addData("X",robot.reletiveXToTarget);
            telemetry.update();

             */

        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(1750,robot.odometers);


        dumper.setPosition(.55);


        robot.wait(750,robot.odometers);








        x = -10;y = -27; finalAngle = Math.toRadians(205);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);



            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            dumper.setPosition(.4);


            armWrist.setPosition(.4);
            arm.setTargetPosition(downPos);
            arm.setPower(0.5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }

        robot.goToPos(-10,-31,Math.toRadians(205),0);

        robot.mecanumDrive(0,0,0,.6);

        //robot.wait(500,robot.odometers);


        inTake.setPower (.75);

        robot.changeSpeed(.3,.3);

        //move to collect
        time = robot.timerInit(1000);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-.2,0,0,1);

        }
        robotHardware.timerInitted = false;

        robot.mecanumDrive(0,0,0,1);


        robot.wait(500,robot.odometers);

        armWrist.setPosition(.4);
        arm.setTargetPosition(30);
        arm.setPower(0.4);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(1500,robot.odometers);

        inTake.setPower (-.75);

        robot.wait(400,robot.odometers);

        armWrist.setPosition(.6);
        arm.setTargetPosition(450);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.changeSpeed(.8,.8);

        //move to dump

        x = 17;y = -4; finalAngle = Math.toRadians(-45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);


            slides.setTargetPosition(4500);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(1000,robot.odometers);


        dumper.setPosition(.55);


        robot.wait(750,robot.odometers);











        x = 0;y = -38; finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);



            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            dumper.setPosition(.4);


            armWrist.setPosition(.4);
            arm.setTargetPosition(downPos);
            arm.setPower(0.5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }


        robot.mecanumDrive(0,0,0,.6);

        //robot.wait(500,robot.odometers);


        inTake.setPower (.75);

        robot.changeSpeed(.3,.3);

        //move to collect
        time = robot.timerInit(1000);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-.2,0,0,1);

        }
        robotHardware.timerInitted = false;

        robot.mecanumDrive(0,0,0,1);


        robot.wait(500,robot.odometers);

        armWrist.setPosition(.6);
        arm.setTargetPosition(30);
        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(1200,robot.odometers);

        inTake.setPower (-.75);

        robot.wait(400,robot.odometers);

        armWrist.setPosition(.5);
        arm.setTargetPosition(450);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.changeSpeed(.8,.8);

        //move to dump

        x = 17;y = -4; finalAngle = Math.toRadians(-45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);


            slides.setTargetPosition(4500);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(1250,robot.odometers);


        dumper.setPosition(.55);


        robot.wait(750,robot.odometers);

        robot.changeSpeed(1,1);
        robot.changeAccuracy(4,Math.toRadians(10));

        x = 5;y = -50; finalAngle = Math.toRadians(0);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);


            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        robot.changeAccuracy(1,Math.toRadians(1));




        time = robot.timerInit(500);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            armWrist.setPosition(0);
            arm.setTargetPosition(00);
            arm.setPower(0.25);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mecanumDrive(1,0,0,1);

        }
        robotHardware.timerInitted = false;

        time = robot.timerInit(1250);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            robot.mecanumDrive(0,-1,0,1);

        }
        robotHardware.timerInitted = false;

        robot.mecanumDrive(0,0,0,.6);

        robot.goToPos(0,-50,finalAngle,Math.toRadians(180));

    }







}
