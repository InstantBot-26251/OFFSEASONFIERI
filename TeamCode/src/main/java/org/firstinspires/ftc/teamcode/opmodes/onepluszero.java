package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Arm2;

@Autonomous(name = "Autonomous Routines for Scoring Specimens")
public class onepluszero extends LinearOpMode { // Change to LinearOpMode
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    Arm2 arm;

    private static final double DRIVE_POWER = 1.0; // Full power to move forward/backward
    private static final int FORWARD_DISTANCE = 1000; // Encoder ticks for forward movement
    private static final int BACKWARD_DISTANCE = 1000; // Encoder ticks for backward movement

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        bl = hardwareMap.get(DcMotorEx.class, "leftRearMotor");
        br = hardwareMap.get(DcMotorEx.class, "rightRearMotor");
        fr = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = new Arm2(hardwareMap);
        // Reset encoders to make sure we start from zero
        resetEncoders();

        waitForStart(); // Wait for the start signal

        // Move forward to the specimen
        moveForward(FORWARD_DISTANCE);

        // Add specimen scoring routine here (Example: actuate arm or claw to score)
        scoreSpecimen();

        // Move backward
        moveBackward(BACKWARD_DISTANCE);
    }

    private void moveForward(int targetPosition) {
        // Set target position for all motors
        setTargetPosition(targetPosition);

        // Set motor power to move forward
        setMotorPowers(DRIVE_POWER);

        // Wait until the robot has reached the target position
        while (opModeIsActive() && motorsAreBusy()) {
            // Optional: Add telemetry for debugging
            telemetry.addData("Moving Forward", "Target: %d", targetPosition);
            telemetry.update();
        }

        // Stop the motors once the robot has reached the target position
        stopMotors();
    }

    private void moveBackward(int targetPosition) {
        // Set target position for all motors, but move backward
        setTargetPosition(-targetPosition);

        // Set motor power to move backward
        setMotorPowers(-DRIVE_POWER);

        // Wait until the robot has reached the target position
        while (opModeIsActive() && motorsAreBusy()) {
            // Optional: Add telemetry for debugging
            telemetry.addData("Moving Backward", "Target: %d", targetPosition);
            telemetry.update();
        }

        // Stop the motors once the robot has reached the target position
        stopMotors();
    }

    private void setTargetPosition(int position) {
        fl.setTargetPosition(fl.getCurrentPosition() + position);
        fr.setTargetPosition(fr.getCurrentPosition() + position);
        bl.setTargetPosition(bl.getCurrentPosition() + position);
        br.setTargetPosition(br.getCurrentPosition() + position);
    }

    private void setMotorPowers(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    private boolean motorsAreBusy() {
        // Check if any motor is still moving toward its target position
        return fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy();
    }

    private void resetEncoders() {
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run to target positions
        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private void stopMotors() {
        setMotorPowers(0);
    }

    private void scoreSpecimen() {
        arm.setSlidePosition(-1507);
    }
}
// Slide -1507, Pivot = -590