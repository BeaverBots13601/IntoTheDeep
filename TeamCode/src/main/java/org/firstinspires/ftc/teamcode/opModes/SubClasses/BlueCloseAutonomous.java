package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Disabled
@Autonomous(name = "Blue Close Autonomous", group = "Competition")
public class BlueCloseAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.BlueClose;
        super.runOpMode();
    }
}