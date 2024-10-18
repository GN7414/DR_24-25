package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="coderDojo")
//@Disabled

public class coderDojo extends LinearOpMode
{
    public Servo armWrist = null;
    public Servo dumper = null;
    public CRServo inTake = null;

    public DcMotor arm = null;
    public DcMotor slides = null;

    public int downPos = 650;
    public static int time1 = 0;
    public static int time2 = 0;
    public static int time3 = 0;

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




        }


        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(.25,.25);

        while(true){
            double[] fake = robot.goToPosSingle(10,0,0,0);
            telemetry.addData("reletiveXToTarget", fake[2]);


            telemetry.update();

        }

        //robot.goToPos(-20,0,Math.toRadians(0),Math.toRadians(0));





    }












    public static boolean boolTimer (int time){
        return outputTime.milliseconds() > time1;

    }

    public static int timerInit (int t, int inittedTi) {

        int ti = 0;

        if (!timerInitted) {

            ti = time1 + t;
            timerInitted = true;

            return ti;

        } else if (boolTimer(inittedTi)) {
            timerInitted = false;

            return inittedTi;

        } else {
            return inittedTi;

        }

    }

    public static int timerInit2 (int t, int inittedTi) {

        int ti = 0;

        if (!timerInitted2) {

            ti = time1 + t;
            timerInitted2 = true;

            return ti;

        } else if (boolTimer(inittedTi)) {
            timerInitted2 = false;

            return inittedTi;

        } else {
            return inittedTi;

        }



    }
    public static int timerInit3 (int t, int inittedTi) {

        int ti = 0;

        if (!timerInitted3) {

            ti = time1 + t;
            timerInitted3 = true;

            return ti;

        } else if (boolTimer(inittedTi)) {
            timerInitted3 = false;

            return inittedTi;

        } else {
            return inittedTi;

        }


    }





}
