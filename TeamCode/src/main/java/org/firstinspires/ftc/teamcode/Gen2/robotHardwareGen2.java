package org.firstinspires.ftc.teamcode.Gen2;
//import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by User on 10/1/2022.
 */
//@Config //We need this for Dashboard to change variables
public class robotHardwareGen2 extends LinearOpMode
{
    //FtcDashboard dashboard = FtcDashboard.getInstance();
    //telemetry = dashboard.getTelemetry(); //This is used to read telemetry on the computer
    //drive motors
    public DcMotor RightOutside = null;
    public DcMotor RightInside = null;
    public DcMotor LeftOutside = null;
    public DcMotor LeftInside = null;


    //odometry encoder objects
    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor perpendicularEncoder = null;


    public DcMotor[] odometers = new DcMotor[3];
    public DcMotor[] drive = new DcMotor[4];
    VoltageSensor ControlHub_VoltageSensor = null;

    public double moveSpeed = 0.5;
    public double turnSpeed = 0.5;

    public double moveAccuracy  = 1;
    public double angleAccuracy = Math.toRadians(1);

    boolean button1 = false;
    boolean button2 = false;

    double power = 0;
    double rightPodPosition = 0;
    double leftPodPosition = 0;
    double aTan = 0;
    double turnPowerRight = 0; //angle of the right stick
    double turnPowerLeft = 0; //angle of the left stick
    double turnEncoder = 0;
    double turnPower = 0;
    double currentAngle = 180;
    double newAngle = 0;
    double rotations = 0;
    double distance = 0;
    double oppositeAngle = 0;
    double oppositeDistance = 0;
    double finalAngle = 0;
    int wheelDirection = 1;
    double encoderTicksPerDegree = 65.743055;//6.40333;//78.38167
    int testing = 0;

    //PID Drive Variables

    public static double DriveF = .1; // = 32767 / maxV      (do not edit from this number)
    public static double DriveP = 0.2; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double DriveI = 0.025;// = 0.1 * P           (fine adjustment of P)
    public static double DriveD = 0.06; // = 0                     (raise to reduce oscillation)

    double DrivePIDCurrentTime = 0;
    double DrivePIDTime = 0;
    double DrivePIDLastTime = 0;
    double DrivePIDError = 0;
    double DrivePIDPreviousError = 0;
    double DrivePIDTotalError = 0;
    double DrivePIDMinIntegral = -1.0;
    double DrivePIDMaxIntegral = 1.0;
    double DrivePIDMotorPower = 0;

    //PID Turning Variables

    public static double TurnF = .15; // = 32767 / maxV      (do not edit from this number)
    public static double TurnP = 0.6; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double TurnI = 0.04; // = 0.1 * P           (fine adjustment of P)
    public static double TurnD = 0.04; // = 0                     (raise to reduce oscillation)

    double TurningPIDCurrentTime = 0;
    double TurningPIDTime = 0;
    double TurningPIDLastTime = 0;
    double TurningPIDError = 0;
    double TurningPIDPreviousError = 0;
    double TurningPIDTotalError = 0;
    double TurningPIDMinIntegral = -1.0;
    double TurningPIDMaxIntegral = 1.0;
    double TurningPIDMotorPower = 0;

    //PID general Variables

    public static double GeneralF = 0.05; // = 32767 / maxV      (do not edit from this number)
    public static double GeneralP = 0.000175; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double GeneralI = 0;// = 0.1 * P           (fine adjustment of P)
    public static double GeneralD = 0.000000; // = 0                     (raise to reduce oscillation)

    double GeneralPIDCurrentTime = 0;
    double GeneralPIDTime = 0;
    double GeneralPIDLastTime = 0;
    double GeneralPIDError = 0;
    double GeneralPIDPreviousError = 0;
    double GeneralPIDTotalError = 0;
    double GeneralPIDMinIntegral = -1.0;
    double GeneralPIDMaxIntegral = 1.0;
    double GeneralPIDMotorPower = 0;

    //PID2 general Variables

    public static double GeneralF2 = 0.001; // = 32767 / maxV      (do not edit from this number)
    public static double GeneralP2 = 0.0025; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double GeneralI2 = 0;// = 0.1 * P           (fine adjustment of P)
    public static double GeneralD2 = 0.0000; // = 0                     (raise to reduce oscillation)

    double GeneralPIDCurrentTime2 = 0;
    double GeneralPIDTime2 = 0;
    double GeneralPIDLastTime2 = 0;
    double GeneralPIDError2 = 0;
    double GeneralPIDPreviousError2 = 0;
    double GeneralPIDTotalError2 = 0;
    double GeneralPIDMinIntegral2 = -1.0;
    double GeneralPIDMaxIntegral2 = 1.0;
    double GeneralPIDMotorPower2 = 0;


    public robotHardwareGen2(HardwareMap ahwMap)
    {

        //drive motors
        RightOutside = ahwMap.dcMotor.get("RightOutside");//Outside is bottom gear on swerve
        RightInside = ahwMap.dcMotor.get("RightInside");//Inside is top gear on swerve
        LeftOutside = ahwMap.dcMotor.get("LeftOutside");//Outside is bottom gear on swerve
        LeftInside  = ahwMap.dcMotor.get("LeftInside");//Inside is top gear on swerve

        RightOutside.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightInside.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftOutside.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftInside.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightOutside.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftOutside.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightOutside.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightInside.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftOutside.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftInside.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightOutside.setPower(0);
        RightInside.setPower(0);
        LeftOutside.setPower(0);
        LeftInside.setPower(0);


        //odometry init (use the motors objects that the odometers are plugged into)
        leftEncoder = LeftOutside;
        rightEncoder = RightOutside;
        perpendicularEncoder = RightOutside;


        odometers[0] = leftEncoder;
        //odometers[1] = rightEncoder;
        odometers[2] = perpendicularEncoder;

        drive[0] = RightOutside;
        drive[1] = RightInside;
        drive[2] = LeftOutside;
        drive[3] = LeftInside;



        ControlHub_VoltageSensor = ahwMap.get(VoltageSensor.class, "Control Hub");
    }

    public void swerveDrive(double forwardDrive, double strafeDrive, double heading, double speed){
        swerveCalculations(forwardDrive, strafeDrive, heading);

        RightOutside.setPower((wheelDirection * power * speed) - turnPowerRight - turnPower * wheelDirection);
        RightInside.setPower((wheelDirection * -power * speed) - turnPowerRight + turnPower * wheelDirection);
        LeftOutside.setPower((wheelDirection * power * speed) + turnPowerLeft + turnPower * wheelDirection);
        LeftInside.setPower((wheelDirection * -power * speed) + turnPowerLeft - turnPower * wheelDirection);
    }
    public void swerveCalculations(double forwardDrive, double strafeDrive, double heading){
        rightPodPosition = -RightOutside.getCurrentPosition();
        leftPodPosition  = LeftOutside.getCurrentPosition();

        power = Math.abs(forwardDrive) + Math.abs(strafeDrive);

        aTan = Math.toDegrees(-Math.atan2(strafeDrive, - forwardDrive)) + 180;

        if (strafeDrive + forwardDrive == 0){
            aTan = 180;
        }

        testing = 0;

        //start of complex things
        currentAngle = rightPodPosition/encoderTicksPerDegree;
        newAngle = aTan; // stick pos

        if (currentAngle > 0){
            rotations = Math.floor(currentAngle/360);
        }
        else {
            rotations = Math.ceil(currentAngle/360);
        }
        newAngle = newAngle + 360 * rotations; // stick pos + 360 for every rotation
        oppositeAngle = newAngle + 180;


        distance = distanceCalculator(currentAngle,newAngle);


        //decide what way is shorter. for example if currentAngle is 350 and new = Angle is 370 then back to 20

        if (distance > distanceCalculator(currentAngle,newAngle + 360)) {
            // decides if new angle should wrap one rotation +
            newAngle = newAngle + 360;
            distance = distanceCalculator(currentAngle,newAngle);
        }
        else if (distance > distanceCalculator(currentAngle,newAngle - 360)) {
            // decides if new angle should wrap one rotation -
            newAngle = newAngle - 360;
            distance = distanceCalculator(currentAngle,newAngle);
        }
        else {
            distance = distanceCalculator(currentAngle,newAngle);
        }

        oppositeDistance = distanceCalculator(currentAngle, oppositeAngle);

        if (oppositeDistance > distanceCalculator(currentAngle,newAngle - 180) && distanceCalculator(currentAngle,newAngle - 180) < 180){
            //distanceCalculator(currentAngle,newAngle - 180)       This checks if it would be faster to go around the circle.
            oppositeAngle = newAngle - 180;
            oppositeDistance = distanceCalculator(currentAngle, oppositeAngle);
            testing = 2;
        }
        if (oppositeDistance > distanceCalculator(currentAngle,newAngle + 180) && distanceCalculator(currentAngle,newAngle + 180) < 180){
            oppositeAngle = newAngle + 180;
            oppositeDistance = distanceCalculator(currentAngle, oppositeAngle);
            testing = 1;
        }


        if(oppositeDistance < distance){
            finalAngle = oppositeAngle;
            wheelDirection = -1;
        } else {
            finalAngle = newAngle;
            wheelDirection = 1;
        }



        //end of complex things

        //uncomment this and comment out the part above to remove fancy turning. good for testing
        //finalAngle = aTan;

        //turn code
        if((aTan < 315 && aTan > 225) || (aTan < 135 && aTan > 45)){
            turnEncoder = heading * 45 * encoderTicksPerDegree;
            turnPower = 0;
        }else{
            turnEncoder = 0;
            turnPower = heading;
        }

        turnPowerRight = odoPID(encoderTicksPerDegree *  finalAngle + turnEncoder, rightPodPosition);
        turnPowerLeft = odoPID(encoderTicksPerDegree *  -finalAngle + turnEncoder, leftPodPosition);
    }

    public double distanceCalculator(double currentAngle, double newAngle){
        double x = 0;
        if(currentAngle < 0 && newAngle >= 0){  //if current = - and new = +
            x = newAngle - currentAngle;
        } else if (currentAngle >= 0 && newAngle < 0){ //if current = + and new = -
            x = currentAngle - newAngle;
        } else {
            x = Math.abs(currentAngle - newAngle);
        }
        return x;
    }


    public void resetDriveEncoders() {
        RightOutside.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightInside.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftOutside.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftInside.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightOutside.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightInside.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOutside.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftInside.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * how to use this code,
     *
     * the cordinant system is a rotated cordinate system with X being forward.
     *
     *       ^ x+
     *       |
     * y+    |
     * <-----O
     *
     *
     *
     * the drive code is written for a mecanum drive, but the localizer will work with any drive train.
     * as long as the odometry pods are set up correctly.
     *
     * there should be 2 forward facing odometry wheels, and one sideways odometry wheel.
     * illistrated bellow:
     *
     *    /--------------\
     *    |     ____     |
     *    |     ----     |
     *    | ||        || |
     *    | ||        || |
     *    |              |
     *    |              |
     *    \--------------/
     *
     *
     *
     *
     * to begin using the code
     * ensure the following lines are in your hardware map:
     * ---------------------------------------------------------------
     * motorRF = ahwMap.dcMotor.get("motorRF");
     * motorLF = ahwMap.dcMotor.get("motorLF");
     * motorRB = ahwMap.dcMotor.get("motorRB");
     * motorLB = ahwMap.dcMotor.get("motorLB");
     *
     * motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     *
     * leftEncoder = motorLF;
     * rightEncoder = motorRF;
     * perpendicularEncoder = motorRB;
     * ---------------------------------------------------------------
     * replace "____Encoder = motor__" with the correct motor (the same motor from the port that the odometry encoder shares)
     *
     **//* the blanks here refer to the words 'left', 'right', and 'perpendicular'*//**
 *
 * replace "odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);" with the chosen hardware map
 *
 *
 **//* the blanks here refer to a program that will be made in the future*/

    /**
     * using ____, tune the values accordingly
     * 1. enter the radius of the odometer wheel as R
     *      to tune:
     *      - push the robot straight forward 100".
     *      - if the distance is not close to 100, adjust R
     *      - repeat till the distance read is close to 100
     *
     *
     * 2. enter the distance between the 2 forward facing encoder wheels in inches as L.
     *      to tune:
     *      - spin 10 times
     *      - if the angle does not read around 0, change L up or down a small amount.
     *      - repeat till the angle reads around 0
     *
     *
     * 3. enter the distance from the middle of a forward facing encoder wheel to the middle of the sideways encoder wheel as B
     *      to tune:
     *      - spin 10 times
     *  *   - if the y distance does not read around 0, change B up or down a small amount.
     *  *   - repeat till the y distance reads around 0
     *
     *
     *
     *  notes:
     * when using the code in auto the refresh() method must be constantly updated
     * use a structure similar to bellow:
     * ---------------------------------------------------------------
     * odometry.goToPos(...);
     * odometry.wait(...);
     * move a servo
     * odometry.wait(...);
     * odometry.goToPos(...);
     * ect.
     * ---------------------------------------------------------------
     * this use of odometry.wait(...); allows the odometry to continue to update. compared to a sleep(...); which pauses all the code for the duration of the sleep.
     *
     *
     *
     * This is the code we used to tune the wheels
     *
     * robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1);
     *
     * robot.refresh(robot.odometers);
     *
     * telemetry.addData("x", robot.GlobalX);
     * telemetry.addData("y", robot.GlobalY);
     * telemetry.addData("heading", robot.GlobalHeading);
     * telemetry.addData("rotations", -robot.GlobalHeading * 57.295 / 360);
     *
     * telemetry.update();
     */

    //odometry constants (tune these)
    double L = 15.375;   //distance between left and right odometers (in inches)
    double B = 4.875;   //distance from center of left/right encoders to the perpendicular encoder (in inches)
    double R = .9425;   //wheel radius (in inches)
    double N = 2000;  //encoder ticks per revoluton
    double inPerTick = 2.0 * Math.PI * R / N;


    /* old values from meet one kept for back up 11/16/2023
    double L = 10;
    double B = 4.6;
    double R = .7514;
    double N = 8192;
    double inPerTick = 2.0 * Math.PI * R / N;
     */

    //changes starting location (in inches)
    public double GlobalX = 0;
    public double GlobalY = 0;
    public double GlobalHeading = 0;

    //track encoder values between loops
    private int currentRightPos = 0;
    private int currentLeftPos = 0;
    private int currentPerpendicularPos = 0;
    private int oldRightPos = 0;
    private int oldLeftPos = 0;
    private int oldPerpendicularPos = 0;

    /**
     * refresh() is the core of the odometry code.
     * calling this method will recalculate the location of the bot, but must be updating regularily inside a loop
     * it will save public values which can be accessed to identify the global position of the robot.
     *
     * enter motors into the array 0.left, 1.right, 2.perpendicular
     *
     * for a good explination of the math behind odometry watch this video:
     * https://www.youtube.com/watch?v=Av9ZMjS--gY
     */

    public void refresh(DcMotor[] odometers)
    {

        //record last loop's encoder reading
        oldRightPos = currentRightPos;
        oldLeftPos = currentLeftPos;
        oldPerpendicularPos = currentPerpendicularPos;

        //record a new encoder reading this loop
        currentRightPos = odometers[0].getCurrentPosition();
        currentLeftPos = odometers[1].getCurrentPosition();
        currentPerpendicularPos = -odometers[2].getCurrentPosition();

        //find the delta encoder values of each encoder
        int dn1 = currentLeftPos - oldLeftPos;
        int dn2 = currentRightPos - oldRightPos;
        int dn3 = currentPerpendicularPos - oldPerpendicularPos;

        //find the delta of x,y,heading reletive to the robot
        double dtheta = inPerTick * (dn2 - dn1) / L;
        double dx = inPerTick * (dn1 + dn2) / 2.0;
        double dy = inPerTick * (dn3 - (dn2 - dn1) * B / L);

        //add the robots movement this loop to the global location
        double theta = (dtheta / 2.0);
        GlobalHeading += dtheta;
        GlobalX -= dx * Math.cos(GlobalHeading) + dy * Math.sin(GlobalHeading);
        GlobalY += dx * Math.sin(GlobalHeading) - dy * Math.cos(GlobalHeading);


        //makes heading 180 to -180
        //angleWrapRad(GlobalHeading);
    }

    // used to mantain angle values between Pi and -Pi
    public double angleWrapRad(double angle)
    {
        while (angle > Math.PI)
        {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI)
        {
            angle += Math.PI * 2;
        }

        return angle;
    }

    //use instead of sleep() in autonomus to keep the location updating
    public void wait(double waitTime, DcMotor[] odometers)
    {
        ElapsedTime time = new ElapsedTime();

        while (time.milliseconds() <= waitTime)
        {
            refresh(odometers);
        }
    }

    //modifier for the speed attributes of the robot when moving in auto
    public void changeSpeed(double mSpeed, double tSpeed){
        moveSpeed = mSpeed;
        turnSpeed = tSpeed;
    }

    //modifier for accuracy variables for the robot in auto
    public void changeAccuracy(double mAccuracy, double aAccuracy){
        moveAccuracy = mAccuracy;
        angleAccuracy = aAccuracy;
    }

    /**
     * this method is the key to using odometry
     * by imputing a location to drive to the robot will calculate an efficient path to the target.
     * if the robot is interfered with, it will recalculate and adjust accordingly
     *
     * at the beginning of the drive the robot will face the target (because mecanums are faster forward than sideways)
     * when it gets within a set distance of the target it will begin turning toward its desired final orientation
     * when it is within its accuracy requirements for the move it will exit the loop and set the motor powers to 0
     *
     * this code is pulled from the basis of pure pursuit.
     * for a better understanding of pure pursuit and if someone wants to improve this code, look into learning how the rest of gluten free's code works here:
     * https://www.youtube.com/watch?v=3l7ZNJ21wMo (5 parts)
     * the code below uses the code explains in parts 1 & 2
     *
     * for the arrays; odometers should be 0.left, 1.right, 2.perpendicular
     * drive motors should be 0.Right front, 1.left front, 2.left back, 3.right back
     *
     * this version will loop until the desired location is reached and then move on
     */
    public void goToPos(double x, double y, double finalAngle, double followAngle)
    {
        //bring in the encoder and motor objects
        //odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        while(Math.abs(x-GlobalX) > moveAccuracy || Math.abs(y-GlobalY) > moveAccuracy || Math.abs(angleWrapRad(finalAngle - GlobalHeading)) > angleAccuracy) {

            //update odometry location
            refresh(odometers);

            double voltComp = (14.0/ControlHub_VoltageSensor.getVoltage()) * (11.0/14.0);

            //math to calculate distances to the target
            double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
            double absoluteAngleToTarget = Math.atan2(x - GlobalX, y - GlobalY);
            double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading-Math.toRadians(90));
            double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
            double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;

            //slow down ensures the robot does not over shoot the target
            double slowDown = Range.clip(odoDrivePID(distanceToTarget,0), 0, moveSpeed);

            //calculate the vector powers for the mecanum math
            double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * slowDown;
            double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * slowDown;

            //when far away from the target the robot will point at the target to get there faster.
            //at the end of the movement the robot will begin moving toward the desired final angle
            double movementTurnPower;
            double reletiveTurnAngle;
            if (distanceToTarget > 6) {
                reletiveTurnAngle = angleWrapRad(reletiveAngleToTarget + followAngle);
                movementTurnPower = Range.clip(odoTurnPID(0, reletiveTurnAngle), -turnSpeed, turnSpeed);
            } else {
                reletiveTurnAngle = angleWrapRad(finalAngle - GlobalHeading);
                movementTurnPower = Range.clip(odoTurnPID(0, reletiveTurnAngle), -turnSpeed, turnSpeed);
            }

            //set the motors to the correct powers to move toward the target
            //mecanumDrive(movementXpower, movementYpower, -movementTurnPower, voltComp);
        }

        //at the end of the movement stop the motors
        drive[0].setPower(0);
        drive[1].setPower(0);
        drive[2].setPower(0);
        drive[3].setPower(0);

    }

    public double goToPosSingle(double x, double y, double finalAngle, double followAngle)
    {
        //bring in the encoder and motor objects
        //odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)


        //update odometry location
        refresh(odometers);

        double voltComp = (14.0/ControlHub_VoltageSensor.getVoltage()) * (11.0/14.0);

        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteAngleToTarget = Math.atan2(x - GlobalX, y - GlobalY);
        double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading - Math.toRadians(90));
        double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
        double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;

        //slow down ensures the robot does not over shoot the target
        double slowDown = Range.clip(odoDrivePID(distanceToTarget,0), 0, moveSpeed);

        //calculate the vector powers for the mecanum math
        double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * slowDown;
        double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * slowDown;

        //when far away from the target the robot will point at the target to get there faster.
        //at the end of the movement the robot will begin moving toward the desired final angle
        double movementTurnPower;
        double reletiveTurnAngle;
        if (distanceToTarget > 6) {
            reletiveTurnAngle = angleWrapRad(reletiveAngleToTarget + followAngle);
            movementTurnPower = Range.clip(odoTurnPID(0, reletiveTurnAngle), -turnSpeed, turnSpeed);
        } else {
            reletiveTurnAngle = angleWrapRad(finalAngle - GlobalHeading);
            movementTurnPower = Range.clip(odoTurnPID(0, reletiveTurnAngle), -turnSpeed, turnSpeed);
        }

        //set the motors to the correct powers to move toward the target
        //mecanumDrive(movementXpower, movementYpower, -movementTurnPower, voltComp);

        return movementTurnPower;
    }

    public double driveToPos(double x, double y){
        //bring in the encoder and motor objects
        //odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)


        //update odometry location
        refresh(odometers);

        double voltComp = (14.0/ControlHub_VoltageSensor.getVoltage()) * (11.0/14.0);

        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteAngleToTarget = Math.atan2(x - GlobalX, y - GlobalY);
        double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading - Math.toRadians(90));
        double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
        double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;

        //slow down ensures the robot does not over shoot the target
        double slowDown = Range.clip(odoDrivePID(distanceToTarget,0), 0, moveSpeed);

        //calculate the vector powers for the mecanum math
        double movementXpower = (-reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * slowDown;
        double movementYpower = (-reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * slowDown;
        if (Double.isNaN(movementYpower)){
            movementYpower = 0;
        }
        if (Double.isNaN(movementXpower)){
            movementXpower = 0;
        }

        //when far away from the target the robot will point at the target to get there faster.
        //at the end of the movement the robot will begin moving toward the desired final angle
        double movementTurnPower;
        double reletiveTurnAngle;
        reletiveTurnAngle = angleWrapRad(reletiveAngleToTarget);
        movementTurnPower = Range.clip(odoTurnPID(0, reletiveTurnAngle), -turnSpeed, turnSpeed);

        //set the motors to the correct powers to move toward the target
        //mecanumDrive(movementXpower, movementYpower, 0, voltComp);

        return slowDown;
    }

    public void turnToAngle(double x, double y, double finalAngle){
        //bring in the encoder and motor objects
        //odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)


        //update odometry location
        refresh(odometers);

        double voltComp = (14.0/ControlHub_VoltageSensor.getVoltage()) * (11.0/14.0);

        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteAngleToTarget = Math.atan2(x - GlobalX, y - GlobalY);
        double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalHeading - Math.toRadians(90));
        double reletiveXToTarget = Math.cos(reletiveAngleToTarget) * distanceToTarget;
        double reletiveYToTarget = Math.sin(reletiveAngleToTarget) * distanceToTarget;

        //slow down ensures the robot does not over shoot the target
        double slowDown = Range.clip(odoDrivePID(0,distanceToTarget), 0, moveSpeed);

        //calculate the vector powers for the mecanum math
        double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * slowDown;
        double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * slowDown;
        if (Double.isNaN(movementYpower)){
            movementYpower = 0;
        }
        if (Double.isNaN(movementXpower)){
            movementXpower = 0;
        }

        //when far away from the target the robot will point at the target to get there faster.
        //at the end of the movement the robot will begin moving toward the desired final angle
        double movementTurnPower;
        double reletiveTurnAngle;
        reletiveTurnAngle = angleWrapRad(finalAngle - GlobalHeading);
        movementTurnPower = Range.clip(odoTurnPID(0, reletiveTurnAngle), -turnSpeed, turnSpeed);


        //set the motors to the correct powers to move toward the target
        //mecanumDrive(movementXpower, movementYpower, movementTurnPower, voltComp);
    }

    /**
     * this method is the key to using odometry
     * by imputing a location to drive to the robot will calculate an efficient path to the target.
     * if the robot is interfered with, it will recalculate and adjust accordingly
     *
     * at the beginning of the drive the robot will face the target (because mecanums are faster forward than sideways)
     * when it gets within a set distance of the target it will begin turning toward its desired final orientation
     *
     * this code is pulled from the basis of pure pursuit.
     * for a better understanding of pure pursuit and if someone wants to improve this code, look into learning how the rest of gluten free's code works here:
     * https://www.youtube.com/watch?v=3l7ZNJ21wMo (5 parts)
     * the code below uses the code explains in parts 1 & 2
     *
     * for the arrays; odometers should be 0.left, 1.right, 2.perpendicular
     * drive motors should be 0.Right front, 1.left front, 2.left back, 3.right back
     *
     * this version will run one calaculation and needs to be used in a loop in the parent autonomus
     */
    /*
    public void goToPosSingle(DcMotor[] odometers, DcMotor[] drive, VoltageSensor ControlHub_VoltageSensor, double x, double y, double finalAngle, double moveSpeed, double turnSpeed, double followAngle)
    {


    }
    */

    public void duelServoController(double target, Servo servoLeft, Servo servoRight){
        servoLeft.setPosition(Math.abs(1-target));
        servoRight.setPosition(target);
    }

    public void servoFineAdjust(Servo s, boolean increase, boolean decrease, double increment){
        if (increase && button1){
            s.setPosition(s.getPosition() + increment);
            button1 = false;
        }
        else if (decrease && button2){
            s.setPosition(s.getPosition() - increment);
            button2 = false;
        }
        else if (!increase && !button1){
            button1 = true;
        }
        else if (!decrease && !button2){
            button2 = true;
        }
    }

    public double odoDrivePID(double target, double current){
        DrivePIDPreviousError = DrivePIDError;
        DrivePIDError = target - current;
        DrivePIDLastTime = DrivePIDCurrentTime;
        DrivePIDCurrentTime = (double) System.nanoTime()/1E9;
        time = DrivePIDCurrentTime - DrivePIDLastTime;
        DrivePIDTotalError += time * DrivePIDError;
        DrivePIDTotalError = DrivePIDTotalError < DrivePIDMinIntegral ? DrivePIDMinIntegral: Math.min(DrivePIDMaxIntegral, DrivePIDTotalError);

        DrivePIDMotorPower = (DriveP * DrivePIDError)
                + (DriveI * DrivePIDTotalError)
                + (DriveD * (DrivePIDError - DrivePIDPreviousError) / time)
                + (DriveF * (DrivePIDError/Math.abs(DrivePIDError)));
        return DrivePIDMotorPower;
    }

    public double odoTurnPID(double target, double current){
        TurningPIDPreviousError = TurningPIDError;
        TurningPIDError = target - current;
        TurningPIDLastTime = TurningPIDCurrentTime;
        TurningPIDCurrentTime = (double) System.nanoTime()/1E9;
        time = TurningPIDCurrentTime - TurningPIDLastTime;
        TurningPIDTotalError += time * TurningPIDError;
        TurningPIDTotalError = TurningPIDTotalError < TurningPIDMinIntegral ? TurningPIDMinIntegral: Math.min(TurningPIDMaxIntegral, TurningPIDTotalError);

        TurningPIDMotorPower = (TurnP * TurningPIDError)
                + (TurnI * TurningPIDTotalError)
                + (TurnD * (TurningPIDError - TurningPIDPreviousError) / time)
                + (TurnF * (Math.signum(TurningPIDError)));
        return TurningPIDMotorPower;
    }

    public double odoPID(double target, double current){
        GeneralPIDPreviousError = GeneralPIDError;
        GeneralPIDError = target - current;
        GeneralPIDLastTime = GeneralPIDCurrentTime;
        GeneralPIDCurrentTime = (double) System.nanoTime()/1E9;
        time = GeneralPIDCurrentTime - GeneralPIDLastTime;
        GeneralPIDTotalError += time * GeneralPIDError;
        GeneralPIDTotalError = GeneralPIDTotalError < GeneralPIDMinIntegral ? GeneralPIDMinIntegral: Math.min(GeneralPIDMaxIntegral, GeneralPIDTotalError);

        GeneralPIDMotorPower = (GeneralP * GeneralPIDError)
                + (GeneralI * GeneralPIDTotalError)
                + (GeneralD * (GeneralPIDError - GeneralPIDPreviousError) / time)
                + (GeneralF * (GeneralPIDError/Math.abs(GeneralPIDError)));
        if (Double.isNaN(GeneralPIDMotorPower)){
            GeneralPIDMotorPower = 0;
        }
        return GeneralPIDMotorPower;
    }

    public double odoPID2(double target, double current){
        GeneralPIDPreviousError2 = GeneralPIDError2;
        GeneralPIDError2 = target - current;
        GeneralPIDLastTime2 = GeneralPIDCurrentTime2;
        GeneralPIDCurrentTime2 = (double) System.nanoTime()/1E9;
        time = GeneralPIDCurrentTime2 - GeneralPIDLastTime2;
        GeneralPIDTotalError2 += time * GeneralPIDError2;
        GeneralPIDTotalError2 = GeneralPIDTotalError2 < GeneralPIDMinIntegral2 ? GeneralPIDMinIntegral2: Math.min(GeneralPIDMaxIntegral2, GeneralPIDTotalError2);

        GeneralPIDMotorPower2 = (GeneralP2 * GeneralPIDError2)
                + (GeneralI2 * GeneralPIDTotalError2)
                + (GeneralD2 * (GeneralPIDError2 - GeneralPIDPreviousError2) / time)
                + (GeneralF2 * (GeneralPIDError2/Math.abs(GeneralPIDError2)));
        if (Double.isNaN(GeneralPIDMotorPower2)){
            GeneralPIDMotorPower2 = 0;
        }
        return GeneralPIDMotorPower2;
    }

    public void runOpMode(){}
}
