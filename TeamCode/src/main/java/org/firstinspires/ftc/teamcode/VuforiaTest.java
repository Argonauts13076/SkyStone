package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@TeleOp(name="Test: VufioriaTest", group="Test")
public class VuforiaTest extends LinearOpMode {

    BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);

        waitForStart();

        hardware.targetsSkyStone.activate();
        while (opModeIsActive()) {
            OpenGLMatrix fieldPos = hardware.fieldTracker.getPosVuf();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    fieldPos.getTranslation().get(0), fieldPos.getTranslation().get(1), fieldPos.getTranslation().get(2));
            telemetry.update();
            sleep(50);
        }
        hardware.targetsSkyStone.deactivate();
    }
}
