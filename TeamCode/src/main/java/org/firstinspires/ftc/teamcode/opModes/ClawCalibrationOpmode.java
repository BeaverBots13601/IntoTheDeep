package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawCalibrationOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        double val = .5;
        while(opModeIsActive()){
            if(gamepad1.left_bumper) val += .01;
            if(gamepad1.right_bumper) val -= .01;
            clawServo.setPosition(val);
            sleep(500);
            telemetry.addData("Val", val);
            telemetry.update();
        }

        // 0.4 open
        // 0.56 closed
    }
}
