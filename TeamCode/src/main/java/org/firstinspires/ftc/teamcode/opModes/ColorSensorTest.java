package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Prox (MM): ", sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Red: ", sensor.getNormalizedColors().red);
            telemetry.addData("Green: ", sensor.getNormalizedColors().green);
            telemetry.addData("Blue: ", sensor.getNormalizedColors().blue);
            telemetry.update();
        }
    }
}
