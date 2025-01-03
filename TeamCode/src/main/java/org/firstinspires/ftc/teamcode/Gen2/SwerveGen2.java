package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RI30HV2;


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
    public boolean upDown = true;
    public boolean out = false;
    public boolean in = true;

    public boolean[] timerArray = new boolean[20];


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

    double time = 0;

    RI30HV2.AutoGrab autoGrab = RI30HV2.AutoGrab.START;

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

        slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);


        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //intake.setPower(Position);

        while (!isStarted() && !isStopRequested()) {
            horizontalExtension.setPosition(.1);
            extensionWrist.setPosition(.5);
            intake.setPower(0);
            bucketWrist.setPosition(.3);
            bucketArm.setPosition(.1);
            SlidesPosition = 100;
            slidesL.setTargetPosition(SlidesPosition);
            slidesL.setPower(.2);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesR.setTargetPosition(SlidesPosition);
            slidesR.setPower(.2);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        while (opModeIsActive()) {

            robot.swerveDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);

            if(gamepad1.dpad_up && buttonDU && SlidesPosition < 2300){
                SlidesPosition = 2300;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonDU = false;
            }

            if(gamepad1.dpad_down && buttonDD && SlidesPosition > 100){
                SlidesPosition = 100;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonDD = false;
            }


            if(!gamepad1.dpad_up && !buttonDU){
                buttonDU = true;
            }

            if(!gamepad1.dpad_down && !buttonDD){
                buttonDD = true;
            }


            if((gamepad1.left_bumper && buttonLB) || timerArray[0] /*Add this to a if to be able to use timer "OR"*/){
                if (gamepad1.left_bumper/*Boolean to start timer*/ && buttonLB) {
                    time = robot.currentTime.milliseconds();//must have button press or will break
                    timerArray[0] = true;
                }


                if (robot.currentTime.milliseconds() > time + 1500) {

                    timerArray[0] = false;//If must be last timer, and must reset boolean when done

                    SlidesPosition = 100;
                    slidesR.setTargetPosition(SlidesPosition);
                    slidesL.setTargetPosition(SlidesPosition);
                    slidesR.setPower(1);
                    slidesL.setPower(1);
                    slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                else if (robot.boolTimer(time + 500) ) {
                    bucketArm.setPosition(.1);
                    bucketWrist.setPosition(.3);
                }
                else{//first thing to happen

                    bucketArm.setPosition(.85);
                    bucketWrist.setPosition(.2);



                }
                buttonLB = false;
                telemetry.addData("booo5100", time);
                telemetry.addData("currentTime", robot.currentTime.milliseconds());
            }

            if(!gamepad1.left_bumper && !buttonLB){
                buttonLB = true;
            }



            //inc extension
            horizontalExtension.setPosition((gamepad1.right_trigger * 0.3)+ 0.1);

            //One button press
            if (gamepad1.left_trigger > .5 && buttonLT) {
                buttonLT = false;

                if (!upDown) {
                    extensionWrist.setPosition(.25);//upPos
                    intake.setPower(0);
                    upDown = false;
                    out = true;

                }
                else if (upDown) { //this closes
                    extensionWrist.setPosition(.9);//downPos
                    intake.setPower(-1);
                    upDown = true;
                }
            }


            else if (gamepad1.left_trigger < .5 && !buttonLT) {
                buttonLT = true;
            }

            if(extensionWrist.getPosition() == .25 && horizontalExtension.getPosition() == .1 && out){
                if (out && in){
                    in = false;
                    time = robot.currentTime.milliseconds();//must have button press or will break
                }

                if (robot.boolTimer(time + 1000) ) {
                    intake.setPower(0);
                    out = false;
                    in = true;

                }
                else if(robot.boolTimer(time + 500) ){
                    intake.setPower(1);

                }
                else{//first thing to happen


                }



            }






















            /*
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



            //Bucket
            //Range is
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



            //placing
            //Wrist + is up
            if(gamepad1.right_trigger > .5 && buttonRT){ //&& APosition < .8){
                bucketArm.setPosition(.85);
                bucketWrist.setPosition(.2);
                buttonRT = false;
            }

            if(gamepad1.left_trigger > .5 && buttonLT){ //&& APosition > .15){
                bucketArm.setPosition(.1);
                bucketWrist.setPosition(.3);
                buttonLT = false;
            }


            if(gamepad1.right_trigger <.5 && !buttonRT){
                buttonRT = true;
            }

            if(gamepad1.left_trigger <.5 && !buttonLT){
                buttonLT = true;
            }


            /*
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

             */






            telemetry.addData("Arm", bucketArm.getPosition());
            telemetry.addData("ArmWrist", bucketWrist.getPosition());
            telemetry.addData("E_Wrist", Position);
            telemetry.addData("H_Extension", HEPosition);
            telemetry.addData("Slides", SlidesPosition);
            telemetry.update();

            /*
            if(gamepad1.a && buttonA && SlidesPosition < 3000){
                SlidesPosition += 100;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(.25);
                slidesL.setPower(.25);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonA = false;
            }

            if(gamepad1.b && buttonB && SlidesPosition > 0){
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


            if(gamepad1.a && Position < 3000){
                Position += 50;
                slidesR.setTargetPosition(Position);
                slidesL.setTargetPosition(Position);
                slidesR.setPower(.25);
                slidesL.setPower(.25);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(gamepad1.b && Position > 50){
                Position -= 50;
                slidesR.setTargetPosition(Position);
                slidesL.setTargetPosition(Position);
                slidesR.setPower(.25);
                slidesL.setPower(.25);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

             */



        }
    }
}
