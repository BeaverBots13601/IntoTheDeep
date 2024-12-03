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

        clawServo.setPosition(.5); // open
        while (!gamepad1.square && opModeIsActive());
        clawServo.setPosition(0); // closed?
        while (!gamepad1.left_bumper && opModeIsActive());
        clawServo.setPosition(1); // closed?
        while (!gamepad1.right_bumper && opModeIsActive());
    }
}
