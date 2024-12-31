package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="SwerveGen2")
//@Disabled

public class SwerveGen2 extends LinearOpMode
{
    public DcMotor slidesR = null;
    public DcMotor slidesL = null;

    public boolean buttonY = true;
    public boolean buttonX = true;
    public boolean buttonLB = true;
    public boolean buttonRB = true;
    public boolean buttonDR = true;
    public boolean buttonDL = true;
    public boolean buttonA = true;
    public boolean buttonB = true;
    public boolean buttonDU = true;
    public boolean buttonDD = true;
    public boolean buttonRT = true;
    public boolean buttonLT = true;

    public boolean[] boolArray = new boolean[20];


    public Servo extensionWrist = null;
    public CRServo intake = null;
    public Servo horizontalExtension = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;

    public int SlidesPosition = 0;

    public double Position =.5;
    public double HEPosition =.1;
    public double APosition = .1;
    public double AWPosition = .35;

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardwareGen2 robot = new robotHardwareGen2(hardwareMap);

        intake = hardwareMap.crservo.get("intake");
        extensionWrist = hardwareMap.servo.get("extensionWrist");
        horizontalExtension = hardwareMap.servo.get("horizontalExtension");
        bucketWrist = hardwareMap.servo.get("bucketWrist");
        bucketArm = hardwareMap.servo.get("bucketArm");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesR.setDirection(DcMotorSimple.Direction.REVERSE);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //intake.setPower(Position);

        while (!isStarted() && !isStopRequested()) {
            horizontalExtension.setPosition(.1);
            extensionWrist.setPosition(.5);
            intake.setPower(0);
            bucketWrist.setPosition(.35);
            bucketArm.setPosition(.1);
        }

        while (opModeIsActive()) {

            //robot.swerveDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);

            //INTAKE
            if(gamepad1.dpad_right && buttonDR){
                //Position += .1;
                intake.setPower(.5);
                buttonDR = false;
            }

            if(gamepad1.dpad_left && buttonDL){
                //Position -= .1;
                intake.setPower(-.5);
                buttonDL = false;
            }


            if(!gamepad1.dpad_right && !buttonDR){
                buttonDR = true;
            }

            if(!gamepad1.dpad_left && !buttonDL){
                buttonDL = true;
            }

            //EXTENSION WRIST
            if(gamepad1.y && buttonY){
                Position = .25;
                extensionWrist.setPosition(Position);
                horizontalExtension.setPosition(.1);
                buttonY = false;
            }

            if(gamepad1.x && buttonX){
                Position = .9;
                extensionWrist.setPosition(Position);
                buttonX = false;
            }


            if(!gamepad1.y && !buttonY){
                buttonY = true;
            }

            if(!gamepad1.x && !buttonX){
                buttonX = true;
            }

            //HORIZONTAL EXTENSION
            if(gamepad1.dpad_up && buttonDU && HEPosition <.4){
                HEPosition += .01;
                horizontalExtension.setPosition(HEPosition);
                buttonDU = false;
            }

            if(gamepad1.dpad_down && buttonDD && HEPosition >.1){
                HEPosition -= .01;
                horizontalExtension.setPosition(HEPosition);
                buttonDD = false;
                intake.setPower(0);
            }


            if(!gamepad1.dpad_up && !buttonDU){
                buttonDU = true;
            }

            if(!gamepad1.dpad_down && !buttonDD){
                buttonDD = true;
            }
            /*
            //Bucket
            //Range is .35-.85
            if(gamepad1.right_bumper && buttonRB && AWPosition < .85){
                AWPosition += .05;
                bucketWrist.setPosition(AWPosition);
                buttonRB = false;
            }

            if(gamepad1.left_bumper && buttonLB &&AWPosition > .35){
                AWPosition -= .05;
                bucketWrist.setPosition(AWPosition);
                buttonLB = false;
            }


            if(!gamepad1.right_bumper && !buttonRB){
                buttonRB = true;
            }

            if(!gamepad1.left_bumper && !buttonLB){
                buttonLB = true;
            }

            */

            /*
            //Arm
            //Higher Number is Up
            if(gamepad1.right_trigger > .5 && buttonRT && APosition < .8){
                APosition += .05;
                bucketArm.setPosition(APosition);
                buttonRT = false;
            }

            if(gamepad1.left_trigger > .5 && buttonLT && APosition > .15){
                APosition -= .05;
                bucketArm.setPosition(APosition);
                buttonLT = false;
            }


            if(gamepad1.right_trigger <.5 && !buttonRT){
                buttonRT = true;
            }

            if(gamepad1.left_trigger <.5 && !buttonLT){
                buttonLT = true;
            }

             */
            if(gamepad1.right_trigger > .5 && buttonRT){ //&& APosition < .8){
                bucketArm.setPosition(.85);
                buttonRT = false;
            }

            if(gamepad1.left_trigger > .5 && buttonLT){ //&& APosition > .15){
                bucketArm.setPosition(.1);
                buttonLT = false;
            }


            if(gamepad1.right_trigger <.5 && !buttonRT){
                buttonRT = true;
            }

            if(gamepad1.left_trigger <.5 && !buttonLT){
                buttonLT = true;
            }

            if(gamepad1.right_bumper && buttonRB){ //&& AWPosition < .85){
                bucketWrist.setPosition(.55);
                buttonRB = false;
            }

            if(gamepad1.left_bumper && buttonLB){ //&&AWPosition > .35){
                bucketWrist.setPosition(.5);
                buttonLB = false;
            }


            if(!gamepad1.right_bumper && !buttonRB){
                buttonRB = true;
            }

            if(!gamepad1.left_bumper && !buttonLB){
                buttonLB = true;
            }






            telemetry.addData("Arm", bucketArm.getPosition());
            telemetry.addData("ArmWrist", bucketWrist.getPosition());
            telemetry.addData("E_Wrist", Position);
            telemetry.addData("H_Extension", HEPosition);
            telemetry.addData("power", SlidesPosition);
            telemetry.update();
            if(gamepad1.a && buttonA && SlidesPosition > -3000){
                SlidesPosition += 100;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(.25);
                slidesL.setPower(.25);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonA = false;
            }

            if(gamepad1.b && buttonB && SlidesPosition < 0){
                SlidesPosition -= 100;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(.25);
                slidesL.setPower(.25);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonB = false;
            }


            if(!gamepad1.a && !buttonA){
                buttonA = true;
            }

            if(!gamepad1.b && !buttonB){
                buttonB = true;
            }


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
