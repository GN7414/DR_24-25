package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="PIDTuner")
//@Disabled
@Config
public class PIDTuner extends LinearOpMode
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

    public static double distance = 25;

    double x = -17,y = 4, finalAngle = Math.toRadians(45);

    public static ElapsedTime outputTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {

        FtcDashboard dashboard = FtcDashboard.getInstance();//How you send telemetry to dashboard
        telemetry = dashboard.getTelemetry();


        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        //dive motors
        //arm = hardwareMap.dcMotor.get("frontArmMotor");
        //slides = hardwareMap.dcMotor.get("slides");


        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //armWrist = hardwareMap.servo.get("intakeWrist");
        //inTake = hardwareMap.crservo.get("intake");
        //armFingerR = hardwareMap.servo.get("frontClawRight");
        //armFingerL = hardwareMap.servo.get("frontClawLeft");
        //slideFingerL = hardwareMap.servo.get("");
        //slideFingerR = hardwareMap.servo.get("");
        //dumper = hardwareMap.servo.get("bucket");

        //slides.setDirection(DcMotor.Direction.REVERSE);



        while(!isStarted() && !isStopRequested()){

            telemetry.addData("movementXpower", 0 );
            telemetry.addData("movementYpower", 0 );
            telemetry.addData("movementTurnPower", 0 );
            telemetry.addData("1", 1 );

            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);
            telemetry.addData("Heading",Math.toDegrees(robot.GlobalHeading));

            telemetry.update();




        }
        robot.changeSpeed(1,1);
        robot.changeAccuracy(1,Math.toRadians(1));


        double x = 25,y = 0, finalAngle = Math.toRadians(0);

        while (true) {
            while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {


                double[] fake = robot.goToPosSingle(x, y, (finalAngle), 0);

                telemetry.addData("movementXpower", fake[4]);
                telemetry.addData("movementYpower", fake[5]);
                telemetry.addData("movementTurnPower",Math.abs(fake[6]));

                telemetry.addData("X",robot.GlobalX);
                telemetry.addData("Y",robot.GlobalY);
                telemetry.addData("Heading",Math.toDegrees(robot.GlobalHeading));

                telemetry.update();

            }
            x = -x;
            while (Math.abs(x - robot.GlobalX) > robot.moveAccuracy || Math.abs(y - robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {


                double[] fake = robot.goToPosSingle(x, y, (finalAngle), 0);

                telemetry.addData("movementXpower", fake[4]);
                telemetry.addData("movementYpower", fake[5]);
                telemetry.addData("movementTurnPower",Math.abs(fake[6]));

                telemetry.addData("X",robot.GlobalX);
                telemetry.addData("Y",robot.GlobalY);
                telemetry.addData("Heading",Math.toDegrees(robot.GlobalHeading));

                telemetry.update();

            }
            x = -x;
            //finalAngle += Math.toRadians(distance);

        }
        //robot.changeAccuracy(1,Math.toRadians(1));
        //robot.changeSpeed(.25,.25);

        //robot.goToPos(10,27,Math.toRadians(155),0);
        //robot.goToPos(10,31,Math.toRadians(155),0);

        /*
        while(true){
            double[] fake = robot.goToPosSingle(0,20,Math.toRadians(0),Math.toRadians(90));
            telemetry.addData("distanceToTarget", fake[0]);
            telemetry.addData("absoluteAngleToTarget", Math.toDegrees(fake[1]));
            telemetry.addData("reletiveXToTarget", fake[2]);
            telemetry.addData("reletiveYToTarget", fake[3]);
            telemetry.addData("movementXpower", fake[4]);
            telemetry.addData("movementYpower", fake[5]);
            telemetry.addData("movementTurnPower", fake[6]);
            telemetry.addData("reletiveTurnAngle", Math.toDegrees(fake[7]));
            telemetry.addData("reletiveAngleToTarget", Math.toDegrees(fake[8]));

            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);
            telemetry.addData("Heading",Math.toDegrees(robot.GlobalHeading));

            telemetry.update();


        }

         */


















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
