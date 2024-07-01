package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name="TeleOpCenterStageFinal_Feb17", group="Linear Opmode")

public class TeleOpCenterStageTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor TopRight = null;
    private DcMotor TopLeft = null;
    private DcMotor BottomRight = null;
    private DcMotor BottomLeft = null;
    private DcMotor ArmMotor = null;
    private DcMotor LinearSlideMotor = null;

    private Servo servoarm = null;
    private Servo planeServo = null;

    private int Padpos = 0;
    private int ArmPos = 0;
    //private DcMotor CarouselMotor = null;
    private DcMotor LiftMotor = null; // port 0
    //private DcMotor IntakeMotor = null;
    private DcMotor ArmMotor2 = null;
    private boolean armMotorPressed = false;
    private boolean holdRequest = false; //armMotor
    private boolean holdRequest2 = false; //armMotor
    private boolean armDownPosition = false;
    private int lmPos = 0;
    private int armCounter = 0;
    private double servoMove = 0.1;

    private double pos = 0.4;
    private double pos2 = 0.4;
    private int slidePos;
    private int armpos;
    //        private DcMotor OutakeMotor = null;
//        private int targetPosition;
    private int armPos;
    private int targetPosition;
    private int targetPosition2;

    private double clicksPerInch = 40; // empirically measured //87.5 - previous value
    private double clicksPerDeg = 21.94; // empirically measured


    static final double COUNTS_PER_MOTOR_REV = 386.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        //Sets up the hardware

        //TopRight
        TopRight = hardwareMap.get(DcMotor.class, "TopRight");
        TopRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);

        //TopLeft
        TopLeft = hardwareMap.get(DcMotor.class, "TopLeft");
        TopLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TopLeft.setDirection(DcMotor.Direction.FORWARD);

        //BottomRight
        BottomRight = hardwareMap.get(DcMotor.class, "BottomRight");
        BottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);

        //BottomLeft
        BottomLeft = hardwareMap.get(DcMotor.class, "BottomLeft");
        BottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);

        //servo
        servoarm = hardwareMap.get(Servo.class, "servoarm"); //write THIS name into the configuration

        planeServo = hardwareMap.get(Servo.class, "planeServo");

        //Linear Slides
        LinearSlideMotor = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Zero Power Behavior
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            checkButtonStatus();
            driveWheels(BottomLeft, BottomRight, TopLeft, TopRight);
            // sliderMotor(LinearSlideMotor);
            slideMotor(LinearSlideMotor);
            servoMove(servoarm);
            planeLaunch(planeServo);
            Arm(ArmMotor);
            setArmDownPosition(ArmMotor);


            telemetry.addData("Pos: ", servoarm.getPosition());
            telemetry.update();




        }
    }

    public void driveWheels(DcMotor BottomLeft, DcMotor BottomRight, DcMotor TopLeft, DcMotor TopRight) {

        double speedAdjust = 7;
//        BottomLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
//        BottomRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));
//        TopLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
//        TopRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));

        BottomLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (speedAdjust / 10));
        BottomRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));
        TopLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (speedAdjust / 10));
        TopRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));
        //  liftArmMotor(ArmMotor,ArmMotor2);
    }

    public void Arm(DcMotor ArmMotor)
    {
        double speed = 7;

        ArmMotor.setPower(-gamepad2.left_stick_y  * 1/2);
    }

    public void checkButtonStatus() {
        if (!gamepad2.right_bumper) {
            armMotorPressed = false;
        }
        if (!gamepad2.left_bumper) {
            armDownPosition = false;
        }
    }


    private void sliderMotor(DcMotor LinearSlideMotor){
        // if (LinearSlideMotor != null) {
        LinearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Padpos = LinearSlideMotor.getCurrentPosition();

        if (gamepad1.x) {
            LinearSlideMotor.setPower(0.3);
            Padpos += 10 * clicksPerInch;
        } else if (gamepad2.y) {
            LinearSlideMotor.setPower(0.3);
            Padpos += 23 * clicksPerInch;
        } else if (gamepad2.a) {
            LinearSlideMotor.setPower(0.3);
            Padpos += 35 * clicksPerInch;
        } else {
            LinearSlideMotor.setPower(0);
        }
        LinearSlideMotor.setTargetPosition(Padpos);
        //   }


    }

    private void setArmDownPosition(DcMotor ArmMotor) {

        if (ArmMotor != null) {
            ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Padpos = LinearSlideMotor.getCurrentPosition();

            if (gamepad2.x) {
                // int position = int(LinearSlideMotor.getCurrentPosition() * clicksPerInch);

                ArmMotor.setPower(0.7);
                armPos = -2017;
                ArmMotor.setTargetPosition(armPos);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Current Position arm: ",":%7d", ArmMotor.getCurrentPosition());
                telemetry.update();


            }

        }
        //
    } // end of slideMotor

    private void slideMotor(DcMotor LinearSlideMotor) {

        if (LinearSlideMotor != null) {
            LinearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Padpos = LinearSlideMotor.getCurrentPosition();

            if (gamepad2.y) {
                // int position = int(LinearSlideMotor.getCurrentPosition() * clicksPerInch);


                LinearSlideMotor.setPower(0.9);
                slidePos = LinearSlideMotor.getCurrentPosition();
                telemetry.addData("Current Position arm: ",":%7d", LinearSlideMotor.getCurrentPosition());
                telemetry.update();


            } else if (gamepad2.a && (LinearSlideMotor.getCurrentPosition() * clicksPerInch) > 0) {

                LinearSlideMotor.setPower(-0.9);
                slidePos = LinearSlideMotor.getCurrentPosition();

                telemetry.addData("Current Position arm: ", ":%7d", LinearSlideMotor.getCurrentPosition());
                telemetry.update();
//                    }
            }
            else {
                LinearSlideMotor.setPower(0);
                LinearSlideMotor.setTargetPosition(slidePos);

            }
            LinearSlideMotor.setTargetPosition(Padpos);
        }
        //
    } // end of slideMotor

    private void servoMove(Servo servoarm) {

        if(gamepad2.dpad_left) {

            //servo CLOSE
            //was 0.9
            servoarm.setDirection(Servo.Direction.FORWARD);
            pos = 0.9;
            servoarm.setPosition(pos);
            telemetry.addData("Pos: ", pos);
            telemetry.update();

        }
        else if(gamepad2.dpad_right) {

            //was 0.7
            servoarm.setDirection(Servo.Direction.REVERSE);
            pos = 0.7;

            //servo OPEN
            servoarm.setPosition(pos);
            telemetry.addData("Pos: ", pos);
            telemetry.update();
        }


        servoarm.setPosition(servoarm.getPosition());
        telemetry.addData("Pos: ", servoarm.getPosition());
        telemetry.update();

        // }


    }
    private void planeLaunch(Servo planeServo)
    {
        if(gamepad1.left_bumper) {
            pos2 = -0.9;
            planeServo.setPosition(pos2);
            telemetry.addData("Pos: ", pos2);
            telemetry.update();
        }
        else if(gamepad1.right_bumper)
        {
            pos2 = 0.7;
            planeServo.setPosition(pos2);
            telemetry.addData("Pos: ", pos2);
            telemetry.update();
        }
        planeServo.setPosition(planeServo.getPosition());
        telemetry.addData("Pos: ", planeServo.getPosition());
        telemetry.update();
    }

}