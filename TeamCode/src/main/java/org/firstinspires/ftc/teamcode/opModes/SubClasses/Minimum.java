package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import java.util.SortedSet;

@Autonomous
public class Minimum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.get(CRServo.class, "servo").setPower(0);

        CRServo goBildaServo = hardwareMap.get(CRServo.class, "servo");

        SortedSet<String> names = hardwareMap.getAllNames(CRServo.class);
        for(String name : names) {
            Log.i("TEAMCODE", name);
        }

        sleep(20 * 1000);
        //CRServo sharkServo = hardwareMap.get(CRServo.class, "shark");


        waitForStart();
        //goBildaServo.setPower(0);
        //sharkServo.setPower(1);

        //goBildaServo.setPower(1);

        while (opModeIsActive());
    }
}
