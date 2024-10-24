package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.ftccommon.external.OnCreate;

import java.util.SortedSet;

@Autonomous
public class Minimum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //ServoController ctrl = hardwareMap.getAll(ServoController.class).get(0);//goBildaServo.getController();

        for (ServoController ctrl : hardwareMap.getAll(ServoController.class)) {
            ctrl.pwmDisable();
        }
        for (CRServo a : hardwareMap.crservo){
            a.setPower(0);
        }
        //ctrl.pwmEnable();

        CRServo goBildaServo = hardwareMap.get(CRServo.class, "shark");

        goBildaServo.setPower(0.5);

        waitForStart();
        //goBildaServo.setPower(0);
        //sharkServo.setPower(1);

        //goBildaServo.setPower(1);

        while (opModeIsActive());
    }
}
