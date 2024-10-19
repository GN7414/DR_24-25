package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="RI30HV2")
//@Disabled

public class RI30HV2 extends LinearOpMode
{
    public boolean buttonB = true;
    public boolean buttonA = true;
    public boolean buttonC = true;
    public boolean openClose = true;

    public Servo armWrist = null;
    public CRServo intake = null;
    public Servo slideFingerR = null;
    public Servo slideFingerL = null;
    public Servo dumper = null;

    public DcMotor arm = null;
    public DcMotor slides = null;

    public int slideEncoder = 0;
    public double speed = 1;

    public int maxSlides = 4500;
    public int downPos = 640;

    public boolean down = false;
    public boolean open = false;

    public boolean x = false;

    double time = 0;
    public boolean timerInit2 = false;


    public enum AutoGrab
    {
        START,
        GRAB_SAMPLE,
        DROP_SAMPLE,
        TOP_SLIDE_POS,
        MIDDLE_SLIDE_POS,
        BOTTOM_SLIDE_POS,
        ARM_TOP_POS,
        ARM_MIDDLE_POS,
        ARM_BOTTOM_POS,
        MANUAL
    }
    AutoGrab autoGrab = AutoGrab.START;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        robot.odo.resetPosAndIMU();

        //dive motors
        arm = hardwareMap.dcMotor.get("frontArmMotor");
        slides = hardwareMap.dcMotor.get("slides");


        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armWrist = hardwareMap.servo.get("intakeWrist");
        intake = hardwareMap.crservo.get("intake");
        //slideFingerL = hardwareMap.servo.get("");
        //slideFingerR = hardwareMap.servo.get("");
        dumper = hardwareMap.servo.get("bucket");

        arm.setDirection(DcMotor.Direction.REVERSE);



        while(!isStarted() && !isStopRequested()){


            //smaller numbers go up higher
            dumper.setPosition(0.4);
            armWrist.setPosition(.5);

            intake.setPower(0);

            arm.setTargetPosition(0);
            arm.setPower(0.175);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        arm.setTargetPosition(100);
        arm.setPower(0.175);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        while (opModeIsActive())
        {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, speed); //normal people
            //robot.mecanumDrive(-gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, speed); //nolan



            //Bucket
            if ((gamepad1.a  && buttonA) || timerInit2){
                // down position
                if(gamepad1.a) {
                    time = robotHardware.currentTime.milliseconds() + 250;
                    timerInit2 = true;
                }


                if(robot.boolTimer(time + 450)){
                    slideEncoder = 0;
                    timerInit2 = false;
                }
                else if (robot.boolTimer(time)){
                    dumper.setPosition(.65);//higher number brings the dumper down
                }
                else {
                    dumper.setPosition(.55);
                }

                buttonA = false;

            }




            if (gamepad1.b){
                dumper.setPosition(.35);
            }
            //Front Claw
            //if (gamepad1.x){
            //    armFingerL.setPosition(0.7);
            //
            //}
            //if (gamepad1.y){
            //    armFingerL.setPosition(0.6);
            //
            //}

            //slide fine addjust
            if (gamepad1.a && buttonA){
                buttonA = false;

            }
            else if (gamepad1.y && buttonB){

                slideEncoder += 200;
                buttonB = false;
                intake.setPower (0);
            }
            if (!gamepad1.a && !buttonA){
                buttonA = true;
            }
            else if (!gamepad1.y && !buttonB){
                buttonB = true;
            }
            if (gamepad1.left_stick_button) {

                downPos = arm.getCurrentPosition() - 5;
                arm.setTargetPosition(downPos);
                arm.setPower(.4);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if (gamepad1.right_stick_button) {

                downPos = arm.getCurrentPosition() + 5;
                arm.setTargetPosition(downPos);
                arm.setPower(.4);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }



            //One button press
            if (gamepad1.right_trigger > .5 && buttonC){
                buttonC = false;
                if (openClose) {
                    autoGrab = AutoGrab.DROP_SAMPLE;
                }
                else if (!openClose) { //this closes
                    autoGrab = AutoGrab.GRAB_SAMPLE;
                }
                open = false;
            }
            else if (gamepad1.right_trigger < .5 && !buttonC) {
                buttonC = true;
            }



            //Arm

            if (gamepad1.dpad_up || robotHardware.timerInitted){//back position
                if (gamepad1.dpad_up){
                    time = robot.timerInit(800);
                }

                dumper.setPosition(.35);

                autoGrab = AutoGrab.ARM_TOP_POS;

                if (robot.boolTimer(time + 500)){
                    autoGrab = AutoGrab.ARM_MIDDLE_POS;
                    robotHardware.timerInitted = false;
                }
                else if (robot.boolTimer(time)){
                    autoGrab = AutoGrab.DROP_SAMPLE;

                }

            }





            if (gamepad1.dpad_right){//collect prep
                autoGrab = AutoGrab.ARM_MIDDLE_POS;
            }
            if (gamepad1.dpad_down){//bottom position for arm
                autoGrab = AutoGrab.ARM_BOTTOM_POS;
                intake.setPower(.75);
            }

            if (gamepad1.left_bumper ) {// 8700 is max

                slideEncoder = maxSlides;

            }
            if (gamepad1.left_trigger > .5 ) {

                slideEncoder = 2100;

            }
            if (gamepad1.right_bumper ) {

                slideEncoder = 0;

            }


            switch (autoGrab)
            {
                case GRAB_SAMPLE:


                    openClose = true;
                    intake.setPower (.75);

                    break;
                case DROP_SAMPLE:

                    openClose = false;
                    intake.setPower (-1);

                    break;
                case TOP_SLIDE_POS:
                    slideEncoder = maxSlides;

                    break;
                case MIDDLE_SLIDE_POS:
                    slideEncoder = 2100;

                    break;
                case BOTTOM_SLIDE_POS:
                    slideEncoder = 0;

                    break;
                case ARM_TOP_POS:

                    armWrist.setPosition(.6);
                    arm.setTargetPosition(50);
                    arm.setPower(0.4);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    break;
                case ARM_MIDDLE_POS:
                    armWrist.setPosition(.5);
                    arm.setTargetPosition(500);
                    arm.setPower(0.25);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setPower(0);


                    break;
                case ARM_BOTTOM_POS:

                    armWrist.setPosition(.4);
                    arm.setTargetPosition(downPos);
                    arm.setPower(0.5);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

            }



















            if (slides.getCurrentPosition() > 6000 || arm.getCurrentPosition() > 550){
                speed = .5;
            }
            else {
                speed = 1;
            }





            //Slides stops
            if (slideEncoder < 0){
                slideEncoder = 0;
            }
            if (slideEncoder > maxSlides){
                slideEncoder = maxSlides;
            }

            //More slides stuff
            //2200 is the max for the slides
            slides.setTargetPosition(slideEncoder);
            slides.setPower(1);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot.odo.update();
            robot.refresh(robot.odometers);


            telemetry.addData("slide encoder",slideEncoder);
            telemetry.addData("slide pos",slides.getCurrentPosition());

            telemetry.addData("",null);

            telemetry.addData("X",robot.GlobalX);
            telemetry.addData("Y",robot.GlobalY);
            telemetry.addData("Heading",robot.GlobalHeading);


            telemetry.addData("",null);

            telemetry.addData("currentRightPos",robot.odometers[0].getCurrentPosition());
            telemetry.addData("currentLeftPos",robot.odometers[1].getCurrentPosition());
            telemetry.addData("currentPerpendicularPos",robot.odometers[2].getCurrentPosition());

            telemetry.addData("armPos",arm.getCurrentPosition());

            telemetry.addData("getPosX", robot.odo.getPosX());
            telemetry.addData("getEncoderX", robot.odo.getEncoderX());
            telemetry.update();


        }

    }
}
