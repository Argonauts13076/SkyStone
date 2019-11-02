package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.*;

@TeleOp(name="Test: Basic Drive Train", group="Test")
public class BasicDriveTrainTeleOp extends LinearOpMode {

    float speedDivisor = 1;

    BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap, telemetry);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;
            boolean gamepad1A = gamepad1.a;
            boolean gamepad1B = gamepad1.b;
            boolean gamepad1X = gamepad1.x;

            if(gamepad1X){
                speedDivisor = 4;
            }
            if(gamepad1A){
                speedDivisor = 2;
            }
            if(gamepad1B) {
                speedDivisor = 1;
            }

            float FrontLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;

            // clip the right/left values so that the values never exceed +/- 1
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            FrontRight = Range.clip(FrontRight, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            // write the values to the motors
            hardware.FrontLeft.setPower(FrontLeft/speedDivisor);
            hardware.FrontRight.setPower(FrontRight/speedDivisor);
            hardware.RearLeft.setPower(BackLeft/speedDivisor);
            hardware.RearRight.setPower(BackRight/speedDivisor);

            sleep(50);
        }
    }
}
