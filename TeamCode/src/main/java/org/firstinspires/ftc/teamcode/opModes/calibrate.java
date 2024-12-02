package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class calibrate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        wristServo.setPosition(.6); // down
        while (!gamepad1.square && opModeIsActive());
        wristServo.setPosition(.45); // 45
        while (!gamepad1.left_bumper && opModeIsActive());
        wristServo.setPosition(.3); // up
        while (!gamepad1.right_bumper && opModeIsActive());
    }
}
