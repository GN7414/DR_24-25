package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="BucketSideAuto")
//@Disabled

public class BucketSideAuto extends LinearOpMode
{
    public Servo extensionWrist = null;
    public CRServo intake = null;
    public Servo horizontalExtension = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;

    public DcMotor slidesR = null;
    public DcMotor slidesL = null;

    public static double time;

    public static boolean timerInitted = false;
    public static boolean timerInitted2 = false;
    public static boolean timerInitted3 = false;
    public static ElapsedTime outputTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException
    {

        robotHardwarePinPoint robot = new robotHardwarePinPoint(hardwareMap);

        robot.resetDriveEncoders();
        robot.odo.resetPosAndIMU();



        while(!isStarted() && !isStopRequested()){


        }

        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(1,1);




        robot.goToPos(10, 0, Math.toRadians(90), Math.toRadians(0));


    }







}
