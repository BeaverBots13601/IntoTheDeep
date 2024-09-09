package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;

@TeleOp(name="Purge Heading File")
public class PurgeHeadingFilePseudoOpmode extends LinearOpMode {
    @Override
    public void runOpMode() {
        File file = new File("robotHeading.txt");
        if(file.exists()) file.delete();
    }
}
