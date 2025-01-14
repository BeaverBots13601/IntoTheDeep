package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.teamcode.rr.InterruptableAction;

// right-front (par0) & left-back (perp) are our drive motors for measuring (port 0 & 3 issue)
// right-rear slide & specimen slide our precise motors (port 0 & 3 issue)

public class SeasonalRobot extends BaseRobot {
    private final DcMotorEx leftRearVerticalArmMotor;
    private final DcMotorEx rightRearVerticalArmMotor;
    private final DcMotorEx specimenSlideMotor;
    // candidate to be moved to base robot

    public SeasonalRobot(LinearOpMode opmode) {
        super(opmode);
        // setup specialized stuff
        leftRearVerticalArmMotor = HardwareMechanism.createDefaultMotor(opmode.hardwareMap, "leftRearVerticalArmMotor");
        leftRearVerticalArmMotor.setDirection(DcMotorSimple.Direction.FORWARD); // hardware thing
        rightRearVerticalArmMotor = HardwareMechanism.createDefaultMotor(opmode.hardwareMap, "rightRearVerticalArmMotor");
        rightRearVerticalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenSlideMotor = HardwareMechanism.createDefaultMotor(opmode.hardwareMap, "specimenSlideMotor");
        specimenSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenFlipServo = HardwareMechanism.setUpServo(opmode.hardwareMap, "specimenFlipServo");
    }
    /*
    This is where all non-standard hardware components should be initialized, stored, and gotten.
    For example, if there is a servo that moves a piece to put a scoring component where it needs to go, but we
    won't need that next year probably, put it here.
    */

    @Deprecated
    public void setRearVerticalArmPower(double speed){
        double limitedSpeed = Math.min(Math.max(speed, -0.70), 0.70); // primary motors
        leftRearVerticalArmMotor.setPower(limitedSpeed);
        rightRearVerticalArmMotor.setPower(limitedSpeed);
    }

    @Deprecated
    public void setSpecimenSlidePower(double speed){
        specimenSlideMotor.setPower(speed);
    }

    @Deprecated
    private final Servo specimenFlipServo;
    @Deprecated
    public void specimenArmToHang(){ specimenFlipServo.setPosition(.73); }
}