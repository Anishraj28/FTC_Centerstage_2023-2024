package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "BlueCloseBackdrop", group = "Linear Opmode")
// @TeleOp(...) is the other common choice
// @Disabled
public class BlueCloseBackdrop extends LinearOpMode {

    // Declare Devices
    DcMotor TopRight = null;
    DcMotor TopLeft = null;
    DcMotor BottomRight = null;
    DcMotor BottomLeft = null;
    DcMotor ArmMotor = null;
    DcMotor ArmMotor2 = null;
    DcMotor LinearSlideMotor = null;

    private int slidePos = 0;

    private int armPos = 0;

    private Servo servoarm = null;

    //        DcMotor OutakeMotor  = null;
//        private Servo WobbleRotate = null;
//        private Servo WobbleClamp = null;
//        private Servo OutakeServo = null;
    //DcMotor TestMotor = null;
    //AnalogInput redEye = null;
    //OpticalDistanceSensor mrOds = null;
    private Servo servo_arm = null;
    private boolean holdRequest = false;
    private DistanceSensor distanceleft;
    private DistanceSensor distanceright;


    // drive motor position variables
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    private int armCounter = 0;
    //private int testPos;

    // operational constants
    private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.3; // medium speed
    private double slow = 0.1; // slow speed
    private double clicksPerInch = 87.5; // empirically measured
    private double clicksPerDeg = 21.94; // empirically measured
    private double lineThreshold = 0.7; // floor should be below this value, line above
    private double redThreshold = 1.9; // red should be below this value, blue above
    private boolean isReleased = false;
    private boolean isLatched = false;
    private boolean isIn = false;



    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 386.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);


        // Initialize the hardware variables.
        TopLeft = hardwareMap.dcMotor.get("TopLeft");
        TopRight = hardwareMap.dcMotor.get("TopRight");
        BottomLeft = hardwareMap.dcMotor.get("BottomLeft");
        BottomRight = hardwareMap.dcMotor.get("BottomRight");
//            OutakeMotor = hardwareMap.dcMotor.get("OutakeMotor");
//            WobbleClamp = hardwareMap.servo.get("WobbleClamp");
//            WobbleRotate = hardwareMap.servo.get("WobbleRotate");
//            OutakeServo = hardwareMap.servo.get("OutakeServo");


        //TestMotor = hardwareMap.dcMotor.get("TestMotor");
        //redEye = hardwareMap.analogInput.get("redEye");
        // mrOds = hardwareMap.opticalDistanceSensor.get("mrOds");
        //mrOds.enableLed(true);

        // The right motors need reversing
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);

        // Set the drive motor run modes:
        TopLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TopRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //OutakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servo_arm = hardwareMap.get(Servo.class, "servoarm"); //write THIS name into the configuration


        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);

        LinearSlideMotor = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        servoarm = hardwareMap.get(Servo.class, "servoarm");

        distanceright = hardwareMap.get(DistanceSensor.class, "DistanceL");
        distanceleft = hardwareMap.get(DistanceSensor.class, "DistanceR");
//
//        ArmMotor2 = hardwareMap.get(DcMotor.class, "ArmMotor2");
//        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ArmMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)

        //servoOpen(servoarm, 0.7);
        waitForStart();


        //distance sensor based autonomous

        if (distanceright.getDistance(DistanceUnit.INCH) < 35) { //right distance sensor, rename later

            //--------------------------------------------------------
//        // autonomous for when robot is close to the backdrop, RIGHT LINE
//        //--------------------------------------------------------
//
            //wake up the robot
            moveForward(1,medium);
            //move forward to scan april tags clearly
            moveForward(20, medium);
            //move forward to deposit purple object at center line
            moveForward(24,medium);
            //turn right to face right line
            turnClockwise(90, medium);
            //move forward to deposit purple pixel on right line
            moveForward(9, medium);
            //strafe right to align with backstage area
            moveRight(24, medium);
            //move backward to park robot in backstage
            moveForward(-38, medium);
            //turn robot around to prepare for teleop
            turnClockwise(180, medium);
        }
        else if(distanceleft.getDistance(DistanceUnit.INCH) < 35) {

            //--------------------------------------------------------
//        // autonomous for when robot is close from the backdrop, LEFT LINE
//        //--------------------------------------------------------
//
            //wake up the robot
            moveForward(1,medium);
            //move forward to scan april tags clearly
            moveForward(20, medium);
            //move forward align with left line
            moveForward(15, medium);
            //turn left to face left line
            turnClockwise(-90, medium);
            //move forward to deposit purple object at center line
            moveForward(9,medium);
            //strafe left to align with backstage
            moveRight(-24, medium);
            //move forward to park in backstage
            moveForward(38, medium);
        }
        else {
            //--------------------------------------------------------
//        // autonomous for when robot is close to the backdrop, CENTER LINE
//        //--------------------------------------------------------
//
            //wake up the robot
            moveForward(1,medium);
            //move forward to scan april tags clearly
            moveForward(20, medium);
            //move forward to deposit purple object at center line
            moveForward(20,medium);
            //move back to get to center point
            moveForward(-28, medium);
            //strafe right to park robot in backstage
            moveRight(38, medium);
            //turn robot around to prepare for teleop
            turnClockwise(-90, medium);
        }

    }

    //this is a 90 degree turn
    //turnClockwise(-23, medium);



    private void moveForward(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();
        //testPos = TestMotor.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos += howMuch * clicksPerInch;
        lrPos += howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;
        //testPos += howMuch * clicksPerInch;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        //TestMotor.setTargetPosition(testPos);

        TopLeft.setPower(speed); //WORKS
        TopRight.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);
        //TestMotor.setPower(speed);

        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //TestMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /*
            TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */
        // wait for move to complete
        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d : %7d : %7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d : %7d : %7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
    }

    private void moveRight(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        TopRight.setPower(speed);
        TopLeft.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);

        // wait for move to complete
        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);

    }

    private void turnClockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();

        // calculate new target
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        TopLeft.setPower(speed);
        TopRight.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);

        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for move to complete
        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
    }

    private void moveToLine(int howMuch, double speed) {
        // howMuch is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        TopLeft.setPower(speed);
        TopRight.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);

        // wait for move to complete
        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {
            //if (mrOds.getLightDetected() > lineThreshold) break;

            // Display it for the driver.
            telemetry.addLine("Move To Line");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);

    }
    private void moveArmMotor(int howMuch, double speed) {

        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        armPos = ArmMotor.getCurrentPosition();

        // calculate new targets
        armPos += howMuch * clicksPerInch;


        // move robot to new position
        ArmMotor.setTargetPosition(slidePos);

        ArmMotor.setPower(speed); //WORKS

        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (ArmMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Arm Motor");
            telemetry.addData("Target", "%7d", armPos);
            telemetry.addData("Actual", "%7d", ArmMotor.getCurrentPosition());
            telemetry.update();
        }

    }

    private void slideMotorUpTest(int howMuch, double speed) {

        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        slidePos = LinearSlideMotor.getCurrentPosition();

        // calculate new targets
        slidePos += howMuch * clicksPerInch;


        // move robot to new position
        LinearSlideMotor.setTargetPosition(slidePos);

        LinearSlideMotor.setPower(speed); //WORKS


        LinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (LinearSlideMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Linear Slide");
            telemetry.addData("Target", "%7d", slidePos);
            telemetry.addData("Actual", "%7d", LinearSlideMotor.getCurrentPosition());
            telemetry.update();
        }

    }

    //In Progress: Need to make it suitable for autonomous
    private void liftArmUp(DcMotor ArmMotor, DcMotor ArmMotor2, Servo servo_arm){
        // holdRequest = false;
      /*  if (servo_arm != null) {
            servo_arm.setDirection(Servo.Direction.FORWARD);
            servo_arm.setPosition(0.5);
        }else {
            telemetry.addData("NO SERVO ACTIVE Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }*/

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmMotor.setPower(0.1);
        ArmMotor2.setPower(-0.1);
        while (armCounter != 400){
            armCounter = armCounter+1;
            telemetry.addData("ARM COUNTER: ", armCounter);
            telemetry.addData("Current Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }


    }

    //In Progress: Need to make it suitable for autonomous
    private void liftArmUp2(DcMotor ArmMotor, DcMotor ArmMotor2, Servo servo_arm){
        // holdRequest = false;
        if (servo_arm != null) {
            servo_arm.setDirection(Servo.Direction.FORWARD);
            servo_arm.setPosition(0.5);
        }else {
            telemetry.addData("NO SERVO ACTIVE Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmMotor.setPower(0.1);
        ArmMotor2.setPower(-0.1);
        while (armCounter != 400){
            armCounter = armCounter+1;
            telemetry.addData("ARM COUNTER: ", armCounter);
            telemetry.addData("Current Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }


    }
    //private void servoO
    private void liftArmDown(DcMotor ArmMotor, DcMotor ArmMotor2,Servo servo_arm){
        //holdRequest = false;

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmMotor.setPower(-0.1);
        ArmMotor2.setPower(0.1);

        if (servo_arm != null) {
            servo_arm.setDirection(Servo.Direction.REVERSE);
            servo_arm.setPosition(-1.0);
        }else {
            telemetry.addData("NO SERVO ACTIVE DOWN Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }


        while (armCounter != 50){
            armCounter = armCounter-1;
            telemetry.addData("ARM COUNTER: ", armCounter);
            telemetry.addData("Current Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }


    }
    //In Progress: Need to make it suitable for autonomous
    private void servoOpen(Servo servo_arm){

        if (servo_arm != null) {
            //servo_arm.setDirection(Servo.Direction.REVERSE);
            servo_arm.setPosition(1.0);
        }else {
            telemetry.addData("NO SERVO ACTIVE DOWN Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }

    }
    //
} // end of Program

