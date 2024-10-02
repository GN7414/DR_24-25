package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="AutoRI30H")
//@Disabled

public class AutoRI30H extends LinearOpMode
{
    public Servo armWrist = null;
    public Servo armFingerR = null;
    public Servo armFingerL = null;
    public Servo slideFingerR = null;
    public Servo slideFingerL = null;
    public Servo dumper = null;

    public DcMotor arm = null;
    public DcMotor slides = null;


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


        armWrist = hardwareMap.servo.get("frontClawWrist");
        armFingerR = hardwareMap.servo.get("frontClawRight");
        armFingerL = hardwareMap.servo.get("frontClawLeft");
        //slideFingerL = hardwareMap.servo.get("");
        //slideFingerR = hardwareMap.servo.get("");
        dumper = hardwareMap.servo.get("bucket");

        slides.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();


        robot.changeAccuracy(1,Math.toRadians(1));
        robot.changeSpeed(1,1);

        double x = 0,y = 0, finalAngle = Math.toRadians(90);

        while(Math.abs(x-robot.GlobalX) > robot.moveAccuracy || Math.abs(y-robot.GlobalY) > robot.moveAccuracy || Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
            robot.goToPosSingle(x, y, finalAngle, 0);

            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);
            telemetry.addData("heading",robot.GlobalHeading);

            telemetry.addData("X",robot.reletiveXToTarget);

            telemetry.update();

        }

        //robot.goToPos(16,-16,0,Math.toRadians(90));




    }
}
