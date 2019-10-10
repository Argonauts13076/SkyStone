package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@TeleOp(name="Test: Cole TeleOp", group="Test")
public class ColeTeleOp extends LinearOpMode {

    ColeHardware hardware = new ColeHardware();

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if(hardware.digitalTouch.getState() == true){
                hardware.testServo.setPosition(ColeHardware.SERVO_FINAL_POSITION);
            }else{
                hardware.testServo.setPosition(ColeHardware.SERVO_INIT_POSITION);
            }
            sleep(50);
        }
    }
}
