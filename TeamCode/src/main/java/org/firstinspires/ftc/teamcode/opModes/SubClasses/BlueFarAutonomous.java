package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Disabled
@Autonomous(name = "Blue Far Autonomous", group = "Competition")
public class BlueFarAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.BlueFar;
        super.runOpMode();
    }
}