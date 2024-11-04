package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

@Autonomous
public class SpecimenSlideCalibrationOp extends LinearOpMode {
    DcMotorEx specimenSlideMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        specimenSlideMotor = hardwareMap.get(DcMotorEx.class, "specimenSlideMotor");
        specimenSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lowerVerticalArm();

        specimenSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        raiseVerticalArm();

        telemetry.addData("Ticks: ", specimenSlideMotor.getCurrentPosition());

        telemetry.update();

        while (opModeIsActive());
    }

    public void raiseVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<>(5);
        float avgChange = 999;
        int lastEncoderPos = specimenSlideMotor.getCurrentPosition();

        specimenSlideMotor.setPower(0.15);

        // run motor as long as not stopped
        while(avgChange >= 3 && opModeIsActive()){
            avgChange = 0;
            while(lastFiveChanges.size() < 5 && opModeIsActive()){
                // get 5 starting values
                lastFiveChanges.add(Math.abs(specimenSlideMotor.getCurrentPosition() - lastEncoderPos));
                lastEncoderPos = specimenSlideMotor.getCurrentPosition();
                sleep(100);
            }
            sleep(50);
            // take out the oldest one & add in a new one
            lastFiveChanges.remove(0);
            lastFiveChanges.add(Math.abs(specimenSlideMotor.getCurrentPosition() - lastEncoderPos));
            lastEncoderPos = specimenSlideMotor.getCurrentPosition();
            for (int num : lastFiveChanges) { avgChange += num; }
            avgChange /= lastFiveChanges.size();
            telemetry.addData("Ticks: ", specimenSlideMotor.getCurrentPosition());
            telemetry.update();
        }

        specimenSlideMotor.setPower(0);
    }
}
