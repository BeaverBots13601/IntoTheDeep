package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class a extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx horizontalArmMotor = hardwareMap.get(DcMotorEx.class, "horizontalArmMotor");
        horizontalArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(!isStopRequested()){
            telemetry.addData("a", horizontalArmMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
