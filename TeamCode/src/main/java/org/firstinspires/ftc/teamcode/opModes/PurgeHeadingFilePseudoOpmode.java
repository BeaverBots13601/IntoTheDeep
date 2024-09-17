package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants;

@TeleOp(name="Purge Heading")
public class PurgeHeadingFilePseudoOpmode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Are you sure you want to purge the heading? (If you're not sure, ask Grayson first.) The current heading is (rads)", constants.ROBOT_HEADING);
        telemetry.update();
        waitForStart();
        constants.ROBOT_HEADING = 0;
    }
}
