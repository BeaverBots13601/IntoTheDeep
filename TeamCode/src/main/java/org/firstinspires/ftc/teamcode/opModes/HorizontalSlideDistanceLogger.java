package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class HorizontalSlideDistanceLogger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx horizontalArmMotor = hardwareMap.get(DcMotorEx.class, "horizontalArmMotor");
        horizontalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        while(!isStopRequested()){
            telemetry.addData("HorizontalSlideDistanceLogger", horizontalArmMotor.getCurrentPosition());
            telemetry.update();

            if (gamepad1.ps){
                horizontalArmMotor.setTargetPosition(1000);
                horizontalArmMotor.setPower(1);
            }

            if (gamepad1.triangle){
                horizontalArmMotor.setTargetPosition(500);
                horizontalArmMotor.setPower(1);
            }

            if (gamepad1.square) {
                horizontalArmMotor.setTargetPosition(1500);
                horizontalArmMotor.setPower(1);
            }

            if (gamepad1.circle){
                horizontalArmMotor.setPower(0);
            }
        }
    }
}
