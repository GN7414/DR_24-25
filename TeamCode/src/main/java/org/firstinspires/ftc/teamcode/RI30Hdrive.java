package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="RI30Hdrive")
@Disabled

public class RI30Hdrive extends LinearOpMode
{
    public boolean buttonB = true;
    public boolean buttonA = true;
    public boolean buttonC = true;
    public boolean openClose = true;
    public Servo armWrist = null;
    public Servo armFingerR = null;
    public Servo armFingerL = null;
    public Servo slideFingerR = null;
    public Servo slideFingerL = null;
    public Servo dumper = null;

    public DcMotor arm = null;
    public DcMotor slides = null;

    public int slideEncoder = 0;
    public double speed = 0.5;

    public int maxSlides = 8500;

    public boolean down = false;
    public boolean open = false;

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



        while(!isStarted() && !isStopRequested()){


            //smaller numbers go up higher
            dumper.setPosition(0.6);
            armFingerL.setPosition(0.4);
            armFingerR.setPosition(0.7);
            armWrist.setPosition(1);


            arm.setTargetPosition(0);
            arm.setPower(0.175);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.wait(1000,robot.odometers);
            //armWrist.setPosition(0);
        }

        arm.setTargetPosition(100);
        arm.setPower(0.175);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive())
        {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, speed); //normal people
            //robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed); //nolan

            //Bucket
            if (gamepad1.a){
                dumper.setPosition(.8);
            }
            if (gamepad1.b){
                dumper.setPosition(.65);
            }
            //Front Claw
            //if (gamepad1.x){
            //    armFingerL.setPosition(0.7);
            //    armFingerR.setPosition(0.3);
            //}
            //if (gamepad1.y){
            //    armFingerL.setPosition(0.6);
            //    armFingerR.setPosition(0.4); //smaller number goes inside
            //}

            //Slides
            if (gamepad1.a && buttonA){
                buttonA = false;
               //slideEncoder -= 200;
            }
            else if (gamepad1.y && buttonB){

                buttonB = false;
                slideEncoder += 200;
            }
            if (!gamepad1.a && !buttonA){
                buttonA = true;
            }
            else if (!gamepad1.y && !buttonB){
                buttonB = true;
            }
            //One button press
            if ((gamepad1.right_trigger > .5 && buttonC) || (open && !down) ){
                buttonC = false;
                if (openClose) {
                    armFingerL.setPosition(0.5);
                    armFingerR.setPosition(0.5);
                    openClose = false;
                }
                else if (!openClose) { //this closes
                    armFingerL.setPosition(0.3); //smaller number closes
                    armFingerR.setPosition(0.7); //larger number closes
                    openClose = true;
                }

                open = false;
            }
            else if (gamepad1.right_trigger < .5 && !buttonC) {
                buttonC = true;
            }
            //Arm
            if (gamepad1.dpad_right){//collect prep

                armWrist.setPosition(.5);
                arm.setTargetPosition(400);
                arm.setPower(0.25);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                open = true;
                openClose = true;
            }
            if (gamepad1.dpad_down){//bottom position for arm

                armWrist.setPosition(.5);

                open = false;
                down = true;

                arm.setTargetPosition(400);
                arm.setPower(0.3);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.sleep(400);

                arm.setTargetPosition(530);
                arm.setPower(0.5);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.dpad_up){//back position

                armWrist.setPosition(.3);
                arm.setTargetPosition(50);
                arm.setPower(0.25);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.sleep(2000);

                open = true;
                down = false;
            }


            if (gamepad1.left_bumper ) {// 8700 is max

                slideEncoder = 8500;

            }
            if (gamepad1.left_trigger > .5 ) {

                slideEncoder = 4250;

            }
            if (gamepad1.right_bumper ) {

                slideEncoder = 0;

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

            telemetry.addData("slide encoder",slideEncoder);
            telemetry.addData("slide pos",slides.getCurrentPosition());
            telemetry.update();


        }
    }
}
