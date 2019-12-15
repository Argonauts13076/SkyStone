package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Argus2.0: Move Forward", group="Argus2.0")
public class MoveForward extends LinearOpMode {

    BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();

    private float[] prevAngle;

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);
        Robot robot = new Robot(hardware);

        waitForStart();
        telemetry.addData("Started", 1);
        telemetry.update();

        hardware.CloseGripper();

        sleep(20000);

        robot.moveAngle(90,0.25);

        sleep(2000);

        robot.moveAngle(0,0);

/*

    robot.moveAngle(0, 0.3);
    sleep(1000);
    robot.moveAngle(Math.PI/4, 0.3);
    sleep(1000);
    robot.moveAngle(Math.PI/2, 0.3);
    sleep(1000);
    robot.moveAngle(Math.PI, 0.3);
    sleep(1000);
    robot.moveAngle((Math.PI * 3)/2.0, 0.3);
    sleep(1000);
*/
/*
    while(opModeIsActive()) {
        robot.motorOn(0, 0.3);
        sleep(1000);
        robot.motorOn(1, 0.3);
        sleep(1000);
        robot.motorOn(2, 0.3);
        sleep(1000);
        robot.motorOn(3, 0.3);
        sleep(1000);
        robot.motorOn(4, 0.3);
        sleep(1000);

    }

 */
    }
}
