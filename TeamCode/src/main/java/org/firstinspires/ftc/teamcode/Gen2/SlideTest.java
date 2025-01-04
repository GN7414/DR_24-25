package org.firstinspires.ftc.teamcode.Gen2;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="SlidesTest")
@Disabled

public class SlideTest extends LinearOpMode
{
    public DcMotor slidesR = null;
    public DcMotor slidesL = null;

    public boolean buttonY = true;
    public boolean buttonX = true;
    public boolean buttonLB = true;
    public boolean buttonRB = true;

    public DcMotor hang = null;

    public int Position = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardwareGen2 robot = new robotHardwareGen2(hardwareMap);

        hang = hardwareMap.dcMotor.get("hang");

        slidesR = hardwareMap.dcMotor.get("RSlides");
        slidesL = hardwareMap.dcMotor.get("LSlides");

        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesL.setDirection(DcMotorSimple.Direction.REVERSE);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();


        waitForStart();

        while (opModeIsActive()) {

            //robot.swerveDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1)

            if(gamepad1.y && buttonY){
                Position += 500;
                hang.setTargetPosition(Position);
                hang.setPower(1);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonY = false;
            }

            if(gamepad1.x && buttonX){
                Position -= 500;
                hang.setTargetPosition(Position);
                hang.setPower(1);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonX = false;
            }

            if(!gamepad1.y && !buttonY){
                buttonY = true;
            }


            if(!gamepad1.x && !buttonX){
                buttonX = true;
            }

            if(gamepad1.right_bumper && buttonRB){
                Position = 0;
                hang.setTargetPosition(Position);
                hang.setPower(1);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonRB = false;
            }

            if(gamepad1.left_bumper && buttonLB){
                Position = 25000;
                hang.setTargetPosition(Position);
                hang.setPower(1);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                buttonLB = false;
            }

            if(!gamepad1.right_bumper && !buttonRB){
                buttonRB = true;
            }


            if(!gamepad1.left_bumper && !buttonLB){
                buttonLB = true;
            }

            telemetry.addData("hang",Position);
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
