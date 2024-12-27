package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="SwerveGen2")
//@Disabled

public class SwerveGen2 extends LinearOpMode
{
    //public DcMotor slidesR = null;
    //public DcMotor slidesL = null;

    public boolean buttonY = true;
    public boolean buttonX = true;
    public boolean buttonLB = true;
    public boolean buttonRB = true;
    public boolean buttonDR = true;
    public boolean buttonDL = true;


    public Servo extensionWrist = null;
    public CRServo intake = null;
    public Servo horizontalExtension = null;

    public double Position = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardwareGen2 robot = new robotHardwareGen2(hardwareMap);

        intake = hardwareMap.crservo.get("intake");
        extensionWrist = hardwareMap.servo.get("extensionWrist");
        horizontalExtension = hardwareMap.servo.get("horizontalExtension");

        //slidesR = hardwareMap.dcMotor.get("RSlides");
        //slidesL = hardwareMap.dcMotor.get("LSlides");

        //slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesL.setDirection(DcMotorSimple.Direction.REVERSE);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //intake.setPower(Position);

        waitForStart();

        while (opModeIsActive()) {

            //robot.swerveDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1)

            if(gamepad1.dpad_right && buttonDR){
                Position += .05;
                intake.setPower(1);
                buttonDR = false;
            }

            if(gamepad1.dpad_left && buttonDL){
                Position -= .05;
                intake.setPower(-1);
                buttonDL = false;
            }


            if(!gamepad1.dpad_right && !buttonDR){
                buttonDR = true;
            }

            if(!gamepad1.dpad_left && !buttonDL){
                buttonDL = true;
            }

            telemetry.addData("power", Position);
            telemetry.update();



//            if(gamepad1.a && Position < 3000){
//                Position += 50;
//                slidesR.setTargetPosition(Position);
//                slidesL.setTargetPosition(Position);
//                slidesR.setPower(.25);
//                slidesL.setPower(.25);
//                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            }
//
//            if(gamepad1.b && Position > 50){
//                Position -= 50;
//                slidesR.setTargetPosition(Position);
//                slidesL.setTargetPosition(Position);
//                slidesR.setPower(.25);
//                slidesL.setPower(.25);
//                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            }



        }
    }
}
