/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "OpenCVTest", group = "Linear Opmode")
public class TeleOpCenterStageOpenCVTest extends LinearOpMode
{
    //declare DC Motors
    DcMotor TopRight = null;
    DcMotor TopLeft = null;
    DcMotor BottomRight = null;
    DcMotor BottomLeft = null;
    DcMotor ArmMotor = null;

    private DcMotor LinearSlideMotor = null;
    private int slidePos = 0;

    private Servo servoarm = null;


    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

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

    OpenCvCamera camera;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of cone
    int left = 10;
    int middle = 12;
    int right = 14;

    // drive motor position variables

    @Override
    public void runOpMode()
    {
        telemetry.setAutoClear(true);

        // Initialize the hardware variables.
        TopLeft = hardwareMap.dcMotor.get("TopLeft");
        TopRight = hardwareMap.dcMotor.get("TopRight");
        BottomLeft = hardwareMap.dcMotor.get("BottomLeft");
        BottomRight = hardwareMap.dcMotor.get("BottomRight");

        TopRight.setDirection(DcMotor.Direction.REVERSE);
        TopLeft.setDirection(DcMotor.Direction.FORWARD);
        BottomRight.setDirection(DcMotor.Direction.REVERSE);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);


        TopLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TopRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LinearSlideMotor = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        servoarm = hardwareMap.get(Servo.class, "servoarm"); //write THIS name into the configuration


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        servoOpen(servoarm, 0.7);
        waitForStart();
        while (isStarted() && !isStopRequested())
        {
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */

      /*  waitForStart();
       if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
            if(tagOfInterest.id == left)
            {
                //what happens if camera detects the left sleeve
                moveForward(5,slow);
                telemetry.addLine("No tag snapshot available, it was never sighted during the start loop :(");
                telemetry.update();

            }
            else if(tagOfInterest.id == middle )
            {
                //what happens if camera detects the middle sleeve
            }
            else if(tagOfInterest.id == right)
            {
                //what happens if camera detects right sleeve
            }
        }
       else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the start loop :(");
           telemetry.update();
        }

       */


        /* Actually do something useful */

        //tagOfInterest.id = middle;
        //tagOfInterest.id = left;


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        // while (opModeIsActive()) {sleep(20);}
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

        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    private void moveLeft(int howMuch, double speed) {
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
            /*
            TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */
        // wait for move to complete
        while (LinearSlideMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Linear Slide");
            telemetry.addData("Target", "%7d", slidePos);
            telemetry.addData("Actual", "%7d", LinearSlideMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        // LinearSlideMotor.setPower(0);
    }

    private void servoOpen(Servo servoarm, double pos) {
        //   if (val == 0)
        //   {
        //  if (servoarm != null) {
        servoarm.setDirection(Servo.Direction.FORWARD);
        servoarm.setPosition(pos);
        //  }


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



}