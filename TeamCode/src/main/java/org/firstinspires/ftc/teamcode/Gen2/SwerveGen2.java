package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RI30HV2;

import java.util.Locale;


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
    public boolean button2A = true;
    public boolean button2B = true;
    public boolean button2X = true;
    public boolean upDown = false;
    public boolean out = false;
    public boolean in = true;
    public boolean B = false;
    public boolean Y = false;
    public boolean X = false;

    public boolean[] timerArray = new boolean[20];


    public Servo extensionWrist = null;
    public CRServo intake = null;
    public Servo horizontalExtension = null;
    public Servo bucketArm = null;
    public Servo bucketWrist = null;
    public Servo turret = null;

    public double WristPosition = 0;
    public int SlidesPosition = 0;

    public double Position =.5;
    public double HEPosition =.1;
    public double APosition = .1;
    public double AWPosition = .35;
    public double SPEED = .5;

    double[] timeArray = new double[20];

    RI30HV2.AutoGrab autoGrab = RI30HV2.AutoGrab.START;

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardwarePinPoint robot = new robotHardwarePinPoint(hardwareMap);

        intake = hardwareMap.crservo.get("intake");
        extensionWrist = hardwareMap.servo.get("extensionWrist");
        horizontalExtension = hardwareMap.servo.get("horizontalExtension");
        bucketWrist = hardwareMap.servo.get("bucketWrist");
        bucketArm = hardwareMap.servo.get("bucketArm");
        turret = hardwareMap.servo.get("turret");

        slidesR = hardwareMap.dcMotor.get("slidesR");
        slidesL = hardwareMap.dcMotor.get("slidesL");

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
         slidesL.setDirection(DcMotorSimple.Direction.REVERSE);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //intake.setPower(Position);

        while (!isStarted() && !isStopRequested()) {
            robot.odo.resetPosAndIMU();
            horizontalExtension.setPosition(.1);
            extensionWrist.setPosition(.0);
            turret.setPosition(.5);
            intake.setPower(0);
            bucketWrist.setPosition(.9);
            bucketArm.setPosition(.125);
            SlidesPosition = 100;
            slidesL.setTargetPosition(SlidesPosition);
            slidesL.setPower(.2);
            slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesR.setTargetPosition(SlidesPosition);
            slidesR.setPower(.2);
            slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, SPEED); //normal people
            //robot.mecanumDrive(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, SPEED); //nolan

            robot.refresh(robot.odometers);


            if(gamepad1.b && buttonB){
                turret.setPosition(robot.TURRET_RIGHT);
                buttonB = false;
            }

            if(!gamepad1.b && !buttonB){
                buttonB = true;

            }


            if(gamepad1.x && buttonX){
                turret.setPosition(robot.TURRET_LEFT);
                buttonX = false;

            }

            if(!gamepad1.x && !buttonX){
                buttonX = true;

            }


            if((gamepad1.y && buttonY)){
                turret.setPosition(robot.TURRET_MIDDLE);
                buttonY = false;

            }

            if(!gamepad1.y && !buttonY){
                buttonY = true;

            }


            if(gamepad1.dpad_up && buttonDU && SlidesPosition < robot.SLIDE_TOP){
                SlidesPosition = 2350;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonDU = false;
                SPEED = .5;
            }

            if(gamepad1.dpad_left && buttonDL && SlidesPosition < robot.SLIDE_MID){
                SlidesPosition = 700;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonDL = false;
                SPEED = .5;
            }

            if(gamepad1.dpad_down && buttonDD && SlidesPosition > 100){
                SlidesPosition = 10000;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonDD = false;
                SPEED = 1;
            }


            if(!gamepad1.dpad_up && !buttonDU){
                buttonDU = true;
            }

            if(!gamepad1.dpad_left && !buttonDL){
                buttonDL = true;
            }

            if(!gamepad1.dpad_down && !buttonDD){
                buttonDD = true;
            }


            if((gamepad1.left_bumper && buttonLB) || timerArray[0] /*Add this to a if to be able to use timer "OR"*/){
                if (gamepad1.left_bumper/*Boolean to start timer*/ && buttonLB) {
                    timeArray[0] = robot.currentTime.milliseconds();//must have button press or will break
                    timerArray[0] = true;
                }


                if (robot.currentTime.milliseconds() > timeArray[0] + 2500) {

                    timerArray[0] = false;//If must be last timer, and must reset boolean when done

                    SlidesPosition = 100;
                    slidesR.setTargetPosition(SlidesPosition);
                    slidesL.setTargetPosition(SlidesPosition);
                    slidesR.setPower(1);
                    slidesL.setPower(1);
                    slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                else if (robot.boolTimer(timeArray[0] + 2000)) {
                    bucketArm.setPosition(robot.BUCKET_ARM_REST);//going to rest position/down
                    bucketWrist.setPosition(robot.BUCKET_WRIST_REST);//rest
                }
                else{//first thing to happen

                    bucketArm.setPosition(robot.BUCKET_ARM_DROP);//dropping
                    bucketWrist.setPosition(robot.BUCKET_WRIST_DROP);//drop

                }
                buttonLB = false;
                telemetry.addData("booo5100", timeArray);
                telemetry.addData("currentTime", robot.currentTime.milliseconds());
            }

            if(!gamepad1.left_bumper && !buttonLB){
                buttonLB = true;
            }



            //inc extension
            horizontalExtension.setPosition((gamepad1.right_trigger * 0.3)+ 0.115);

            //One button press
            if (gamepad1.left_trigger > .5 && buttonLT) {
                buttonLT = false;

                if (!upDown) {
                    extensionWrist.setPosition(robot.WRIST_DROP);//upPos
                    intake.setPower(0);
                    upDown = true;
                    out = true;
                    SPEED = 1;

                }
                else if (upDown) {
                    extensionWrist.setPosition(robot.WRIST_PICKUP);//downPos
                    intake.setPower(-1);
                    upDown = false;
                    out = false;
                    SPEED = .25;
                }
            }
            else if (gamepad1.left_trigger < .5 && !buttonLT) {
                buttonLT = true;
            }



            if(upDown && horizontalExtension.getPosition() < .13){
                telemetry.addData("MainThing", true);
                if (out && in){
                    telemetry.addData("Thing", true);
                    in = false;
                    timeArray[1] = robot.currentTime.milliseconds();//must have button press or will break
                }

                if (robot.boolTimer(timeArray[1] + 1500) ) {
                    intake.setPower(0);
                    out = false;
                    in = true;
                    extensionWrist.setPosition(.5);


                }
                else if(robot.boolTimer(timeArray[1] + 400) ){
                    intake.setPower(1);

                }
                else{//first thing to happen
                }

            }

            if(gamepad2.dpad_up){

                slidesR.setTargetPosition(2300);
                slidesL.setTargetPosition(2300);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(gamepad2.dpad_down){

                slidesR.setTargetPosition(1600);
                slidesL.setTargetPosition(1600);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(gamepad2.a && button2A){//slides fine adjust

                SlidesPosition += 100;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                button2A = false;

            }

            if(!gamepad2.a && !button2A){

                button2A = true;
            }

            if(gamepad2.b && button2B){

                SlidesPosition -= 100;
                slidesR.setTargetPosition(SlidesPosition);
                slidesL.setTargetPosition(SlidesPosition);
                slidesR.setPower(1);
                slidesL.setPower(1);
                slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                button2B = false;

            }

            if(!gamepad2.b && !button2B){

                button2B = true;
            }

            if(gamepad2.x && button2X){//resetting slide's encoder

                slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                button2X = false;

            }

            if(!gamepad2.x && !button2X){

                button2X = true;
            }

            if((gamepad1.right_bumper && buttonRB)/** || timerArray[1]**/ /*Add this to a if to be able to use timer "OR"*/){
                intake.setPower(-intake.getPower());
                buttonRB = false;

            }

            if(!gamepad1.right_bumper && !buttonRB){
                buttonRB = true;
            }

            robot.refresh(robot.odometers);

            telemetry.addData("turret", turret.getPosition());
            telemetry.addData("Arm", bucketArm.getPosition());
            telemetry.addData("ArmWrist", bucketWrist.getPosition());
            telemetry.addData("E_Wrist", Position);
            telemetry.addData("H_Extension", HEPosition);
            telemetry.addData("Slides", SlidesPosition);
            telemetry.addData("extension Wrist", extensionWrist.getPosition());
            telemetry.addData("Horizontal extension", horizontalExtension.getPosition());

            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);

            telemetry.addData("X",robot.odo.getEncoderX());
            telemetry.addData("Y",robot.odo.getEncoderY());

            telemetry.addData("slideR",slidesR.getCurrentPosition());
            telemetry.addData("slideL",slidesL.getCurrentPosition());

            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();




        }
    }
}
