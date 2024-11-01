package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="newAuto")
@Disabled

public class newAuto extends LinearOpMode
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
        arm.setTargetPosition(450);
        arm.setPower(0.25);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(.8,.8);

        double x = -17,y = 4, finalAngle = Math.toRadians(45);

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











        x = 0;y = 50; finalAngle = Math.toRadians(0);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {

            robot.goToPosSingle(x, y, finalAngle, 0);



            slides.setTargetPosition(200);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            dumper.setPosition(.4);





        }
        armWrist.setPosition(0);
        arm.setTargetPosition(0);
        arm.setPower(0.4);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.mecanumDrive(0,0,0,.6);

        robot.wait(2000,robot.odometers);
    }







}
