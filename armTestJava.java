package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "armmove (Blocks to Java)")
public class armTestJava extends LinearOpMode {

    private DcMotor ArmMotor;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int armposition;

        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            armposition = 0;
            ArmMotor.setTargetPosition(0);
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.dpad_up) {
                    ArmMotor.setTargetPosition(armposition + 800);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(1);
                } else if (gamepad1.dpad_down) {
                    ArmMotor.setTargetPosition(armposition - 75);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(0.5);
                }
                telemetry.update();
            }
        }
    }
}