package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

@Autonomous(name = "Autonomous Routines for Scoring Specimens")
public class onepluszero extends OpMode {
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        bl = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        br = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        fr = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        long timeLimit;
    }

    public void moveForward(long timeLimit) {
        long time = System.currentTimeMillis();
        if (time < timeLimit) {
        fl.setPower(1);
        fr.setPower(1);
        br.setPower(1);
        bl.setPower(1);
    }

//    public boolean isMoving() {
//        if (fl.isBusy() || fr.isBusy() || br.isBusy() || bl.isBusy()) {
//            return true;
//        }
//        else {
//            return false;
//        }
//    }
} }
