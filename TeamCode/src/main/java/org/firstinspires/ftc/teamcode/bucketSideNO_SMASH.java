package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="bucketSideNO_SMASH")
//@Disabled

public class bucketSideNO_SMASH extends LinearOpMode
{
    public Servo armWrist = null;
    public Servo dumper = null;
    public CRServo inTake = null;

    public DcMotor arm = null;
    public DcMotor slides = null;

    public int downPos = 660;
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

        arm.setDirection(DcMotor.Direction.REVERSE);



        while(!isStarted() && !isStopRequested()){

            dumper.setPosition(.4);
            //armWrist.setPosition(.5);
            inTake.setPower(0);


            arm.setTargetPosition(0);
            arm.setPower(0.175);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }










        armWrist.setPosition(.4);
        arm.setTargetPosition(robot.ARM_MID);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(.8,.8);

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

        double x = -17,y = 4, finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slides.setTargetPosition(4400);
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

        robot.wait(950,robot.odometers);

        armWrist.setPosition(robot.WRIST_TOP);

        dumper.setPosition(.6);


        robot.wait(750,robot.odometers);
        robot.changeSpeed(1,1);




        /**
         *
         * first collection
         *
         *
         *
         *
         *
         *
         */

        x = 10.75;y = 24; finalAngle = Math.toRadians(145);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(145));



            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            dumper.setPosition(.4);



            arm.setTargetPosition(downPos);
            arm.setPower(0.4);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }

        robot.changeSpeed(.4,.4);

        armWrist.setPosition(robot.WRIST_LOW);
        arm.setTargetPosition(robot.ARM_LOW);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.goToPos(10,31,Math.toRadians(-205),0);

        robot.mecanumDrive(0,0,0,.6);

        //robot.wait(500,robot.odometers);


        inTake.setPower (.75);

        robot.changeSpeed(.3,.3);

        //move to collect
        time = robot.timerInit(500);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-.3,0,0,1);

        }
        robotHardware.timerInitted = false;

        robot.mecanumDrive(0,0,0,1);


        robot.wait(500,robot.odometers);

        armWrist.setPosition(robot.WRIST_TOP);
        arm.setTargetPosition(robot.ARM_TOP);
        arm.setPower(0.75);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(700,robot.odometers);

        inTake.setPower (-.75);

        robot.wait(500,robot.odometers);

        armWrist.setPosition(robot.WRIST_TOP);
        arm.setTargetPosition(robot.ARM_MID);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.changeSpeed(.8,.8);

        //move to dump

        x = -17;y = 4; finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slides.setTargetPosition(4400);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(700,robot.odometers);


        dumper.setPosition(.6);


        robot.wait(750,robot.odometers);






        /**
         *
         *
         *
         *
         *
         *
         * going to the second sample to collect
         *
         */

        robot.changeSpeed(1,1);

        x = 3;y = 37.5; finalAngle = Math.toRadians(180);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));



            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            dumper.setPosition(.4);


            armWrist.setPosition(robot.WRIST_LOW);
            arm.setTargetPosition(downPos);
            arm.setPower(0.4);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }

        robot.changeSpeed(.4,.4);

        arm.setTargetPosition(downPos);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.goToPos(10,31,Math.toRadians(-205),0);

        robot.mecanumDrive(0,0,0,.6);

        //robot.wait(500,robot.odometers);


        inTake.setPower (.75);

        robot.changeSpeed(.3,.3);

        //move to collect
        time = robot.timerInit(500);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-.3,0,0,1);

        }
        robotHardware.timerInitted = false;

        robot.mecanumDrive(0,0,0,1);


        robot.wait(300,robot.odometers);

        armWrist.setPosition(robot.WRIST_TOP);
        arm.setTargetPosition(robot.ARM_TOP);
        arm.setPower(0.75);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(700,robot.odometers);

        inTake.setPower (-.75);

        robot.wait(500,robot.odometers);

        armWrist.setPosition(robot.WRIST_TOP);
        arm.setTargetPosition(robot.ARM_MID);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.changeSpeed(.8,.8);

        //move to dump

        x = -17;y = 4; finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slides.setTargetPosition(4400);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(800,robot.odometers);


        dumper.setPosition(.6);


        robot.wait(750,robot.odometers);




        /**
         *
         *
         *
         *
         *
         *
         * third collection
         *
         */

        robot.changeSpeed(1,1);

        robot.changeAccuracy(6,Math.toRadians(5));

        x = -7.4;y = 17.3; finalAngle = Math.toRadians(90);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(0));



            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            dumper.setPosition(.4);


            armWrist.setPosition(robot.WRIST_LOW);
            arm.setTargetPosition(downPos);
            arm.setPower(0.5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        robot.changeAccuracy(.5,Math.toRadians(1));

        x = -3;y = 37; finalAngle = Math.toRadians(180);
        robot.goToPos(x, y, finalAngle, Math.toRadians(0));


        robot.mecanumDrive(0,0,0,.6);



        arm.setTargetPosition(downPos);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.wait(500,robot.odometers);


        inTake.setPower(.75);

        robot.changeSpeed(.8,.8);

        //second move to collect
        time = robot.timerInit(600);
        while (!robot.boolTimer(time)){

            robot.refresh(robot.odometers);

            robot.mecanumDrive(-.3,0,0,1);

        }
        robotHardware.timerInitted = false;

        robot.mecanumDrive(0,0,0,1);


        robot.wait(200,robot.odometers);

        robot.changeAccuracy(1,Math.toRadians(1));

        robot.goToPos(-7, 38, finalAngle, Math.toRadians(180));


        armWrist.setPosition(robot.WRIST_TOP);
        arm.setTargetPosition(robot.ARM_TOP);
        arm.setPower(0.75);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wait(700,robot.odometers);

        inTake.setPower (-.75);

        robot.wait(450,robot.odometers);

        armWrist.setPosition(robot.WRIST_MID);
        arm.setTargetPosition(robot.ARM_MID);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.changeSpeed(.8,.8);

        //second move to dump

        x = -17;y = 4; finalAngle = Math.toRadians(45);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));


            slides.setTargetPosition(4400);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        robot.mecanumDrive(0,0,0,.6);

        robot.wait(1000,robot.odometers);


        dumper.setPosition(.6);


        robot.wait(750,robot.odometers);

        robot.changeSpeed(1,1);
        robot.changeAccuracy(4,Math.toRadians(10));

       // x = 74;y = 3.75; finalAngle = Math.toRadians(0);

       // while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

       //     robot.goToPosSingle(x, y, finalAngle, 0);


            slides.setTargetPosition(0);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            arm.setTargetPosition(120);
            arm.setPower(0.4);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       // }
        robot.changeAccuracy(1,Math.toRadians(1));




        /*
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

         */


        /**
         * parking
         *
         *
         *
         *
         *
         *
         */

        //x = 0;y = 50; finalAngle = Math.toRadians(0);
        //while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
        //robot.goToPosSingle(x, y, finalAngle, Math.toRadians(180));
        robot.wait(2000,robot.odometers);
        armWrist.setPosition(0.4);
        arm.setTargetPosition(0);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //}
        //robot.mecanumDrive(0,0,0,.6);

        robot.wait(1000,robot.odometers);

        robot.mecanumDrive(0,0,0,.6);
    }







}
