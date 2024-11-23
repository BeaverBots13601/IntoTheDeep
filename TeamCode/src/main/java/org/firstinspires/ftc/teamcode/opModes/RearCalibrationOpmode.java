package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SeasonalRobot;

import java.util.ArrayList;

@Disabled
@Autonomous
public class RearCalibrationOpmode extends LinearOpMode {
    DcMotorEx leftVerticalArmMotor;
    DcMotorEx rightVerticalArmMotor;
    @Override
    public void runOpMode() {
        waitForStart();
        leftVerticalArmMotor = hardwareMap.get(DcMotorEx.class, "leftVerticalArmMotor");
        leftVerticalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVerticalArmMotor = hardwareMap.get(DcMotorEx.class, "rightVerticalArmMotor");
        rightVerticalArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVerticalArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lowerVerticalArm();

        leftVerticalArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerticalArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        raiseVerticalArm();

        telemetry.addData("Left Arm Ticks: ", leftVerticalArmMotor.getCurrentPosition());
        telemetry.addData("Right Arm Ticks (important): ", rightVerticalArmMotor.getCurrentPosition());
        telemetry.addData("Average: ", (leftVerticalArmMotor.getCurrentPosition() + rightVerticalArmMotor.getCurrentPosition()) / 2);

        telemetry.update();

        while (opModeIsActive());
    }

    public void raiseVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<>(5);
        float avgChange = 999;
        int lastEncoderPos = rightVerticalArmMotor.getCurrentPosition();

        leftVerticalArmMotor.setPower(0.15);
        rightVerticalArmMotor.setPower(0.15);

        // run motor as long as not stopped
        while(avgChange >= 3 && opModeIsActive()){
            avgChange = 0;
            while(lastFiveChanges.size() < 5 && opModeIsActive()){
                // get 5 starting values
                lastFiveChanges.add(Math.abs(rightVerticalArmMotor.getCurrentPosition() - lastEncoderPos));
                lastEncoderPos = rightVerticalArmMotor.getCurrentPosition();
                sleep(100);
            }
            sleep(50);
            // take out the oldest one & add in a new one
            lastFiveChanges.remove(0);
            lastFiveChanges.add(Math.abs(rightVerticalArmMotor.getCurrentPosition() - lastEncoderPos));
            lastEncoderPos = rightVerticalArmMotor.getCurrentPosition();
            for (int num : lastFiveChanges) { avgChange += num; }
            avgChange /= lastFiveChanges.size();
            telemetry.addData("Left Arm Ticks: ", leftVerticalArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Ticks (important): ", rightVerticalArmMotor.getCurrentPosition());
            telemetry.addData("Average: ", (leftVerticalArmMotor.getCurrentPosition() + rightVerticalArmMotor.getCurrentPosition()) / 2);
            telemetry.update();
        }

        leftVerticalArmMotor.setPower(0);
        rightVerticalArmMotor.setPower(0);
    }
}