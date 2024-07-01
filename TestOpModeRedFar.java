import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CompetitionRedLeft;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.*;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CompetitionBlueLeft;

@Autonomous(name = "Test Detection Red Far")
public class TestOpModeRedFar extends LinearOpMode {

    private CompetitionRedLeft detector;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    DcMotor TopRight = null;
    DcMotor TopLeft = null;
    DcMotor BottomRight = null;
    DcMotor BottomLeft = null;
    DcMotor ArmMotor = null;
    DcMotor ArmMotor2 = null;

    int armPos = 0;

    //        DcMotor OutakeMotor  = null;
//        private Servo WobbleRotate = null;
//        private Servo WobbleClamp = null;
//        private Servo OutakeServo = null;
    //DcMotor TestMotor = null;
    //AnalogInput redEye = null;
    //OpticalDistanceSensor mrOds = null;
    private Servo servoarm = null;
    private boolean holdRequest = false;


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
        detector = new CompetitionRedLeft(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), detector);

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Detected Position", detector.position);
            telemetry.update();
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
            BottomLeft.setDirection(DcMotor.Direction.REVERSE);

            // Set the drive motor run modes:
            TopLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TopRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //OutakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            servoarm = hardwareMap.get(Servo.class, "servoarm"); //write THIS name into the configuration


//        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
//        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ArmMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        ArmMotor2 = hardwareMap.get(DcMotor.class, "ArmMotor2");
//        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ArmMotor2.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses PLAY)

            waitForStart();
            //close the servo
            // servoOpen(servo_arm);
//        liftArmUp(ArmMotor, ArmMotor2,servo_arm);

            //--------------------------------------------------------
            // autonomous for when robot is far from the backdrop, CENTER LINE
            //--------------------------------------------------------

            if(detector.position == 1) {

                //center is -2 right
                moveRight(-2, medium);

// change to 20
                moveForward(20, medium);

               //code to open servo
                sleep(1000);
                servoOpen(servoarm, 0.7);
                sleep(1000);

                moveForward(-2, medium);
                moveRight(75, medium);



            }
            else if(detector.position == 2) {
                //moveRight(10, medium);

                moveForward(20, medium);



                moveForward(-2, medium);
                moveRight(10, medium);
                //code to open servo
                sleep(1000);
                servoOpen(servoarm, 0.7);
                sleep(1000);

                moveRight(64, medium);
            }
            else if (detector.position == 3) {
                moveRight(-10, medium);
                moveForward(12, medium);

                //code to open servo
                sleep(1000);
                servoOpen(servoarm, 0.7);
                sleep(1000);

                //moveForward(-2, medium);
                moveRight(80, medium);
            }
            else
            {
                moveForward(-3, medium);
            }

        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
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

        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
    private void servoOpen(Servo servo_arm, double pos){

        if (servo_arm != null) {
            //servo_arm.setDirection(Servo.Direction.REVERSE);
            servoarm.setDirection(Servo.Direction.FORWARD);
            servo_arm.setPosition(pos);
        }else {
            telemetry.addData("NO SERVO ACTIVE DOWN Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }

    }
}



