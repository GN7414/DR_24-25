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

        slides.setDirection(DcMotor.Direction.REVERSE);



        while(!isStarted() && !isStopRequested()){

            dumper.setPosition(.6);
            //armWrist.setPosition(.5);
            inTake.setPower(0);


            arm.setTargetPosition(0);
            arm.setPower(0.175);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }


        armWrist.setPosition(.5);
        arm.setTargetPosition(450);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(.6,.6);

        double x = 17,y = -4, finalAngle = Math.toRadians(-45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);


            slides.setTargetPosition(8500);
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

        robot.wait(3000,robot.odometers);


        dumper.setPosition(.8);


        robot.wait(1500,robot.odometers);

        x = -10;y = -28; finalAngle = Math.toRadians(205);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);



            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            dumper.setPosition(0.6);


            armWrist.setPosition(.475);
            arm.setTargetPosition(downPos);
            arm.setPower(0.5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        robot.mecanumDrive(0,0,0,.6);

        //robot.wait(500,robot.odometers);


        inTake.setPower (.75);

        robot.changeSpeed(.2,.2);

        //move to collect
        time = robot.timerInit(1000);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-.2,0,0,1);

        }
        robotHardware.timerInitted = false;

        robot.mecanumDrive(0,0,0,1);


        robot.wait(1000,robot.odometers);

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

        robot.wait(400,robot.odometers);

        robot.changeSpeed(.6,.6);

        //move to dump

        x = 17;y = -4; finalAngle = Math.toRadians(-45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);


            slides.setTargetPosition(8500);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(2500,robot.odometers);


        dumper.setPosition(.8);


        robot.wait(1500,robot.odometers);








    }







}
