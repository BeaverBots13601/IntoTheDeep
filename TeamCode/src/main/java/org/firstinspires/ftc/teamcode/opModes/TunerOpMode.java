package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TunerOpMode extends LinearOpMode {
    public void runOpMode(){
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo specimenFlipServo = hardwareMap.get(Servo.class, "specimenFlipServo");
        Servo specimenClawServo = hardwareMap.get(Servo.class, "specimenClawServo");

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.ps){
                wristServo.setPosition(0.5); // .62 90deg; .5 45deg; .27 down
                specimenFlipServo.setPosition(0.5); // about midway;
                specimenClawServo.setPosition(0.5); // .52 closed; .23 open
            }

            if (gamepad1.dpad_up) wristServo.setPosition(wristServo.getPosition() + 0.01);
            if (gamepad1.dpad_down) wristServo.setPosition(wristServo.getPosition() - 0.01);

            if (gamepad1.circle) specimenFlipServo.setPosition(specimenFlipServo.getPosition() + 0.01);
            if (gamepad1.square) specimenFlipServo.setPosition(specimenFlipServo.getPosition() - 0.01);

            if (gamepad1.left_bumper) specimenClawServo.setPosition(specimenClawServo.getPosition() + 0.01);
            if (gamepad1.right_bumper) specimenClawServo.setPosition(specimenClawServo.getPosition() - 0.01);

            telemetry.addData("wrist servo: ", wristServo.getPosition());
            telemetry.addData("specimen flip servo: ", specimenFlipServo.getPosition());
            telemetry.addData("specimen claw servo: ", specimenClawServo.getPosition());
            telemetry.update();

            sleep(50);
        }
    }
}
