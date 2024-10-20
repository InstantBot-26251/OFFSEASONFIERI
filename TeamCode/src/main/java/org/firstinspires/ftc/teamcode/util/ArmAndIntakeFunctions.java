package org.firstinspires.ftc.teamcode.util;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ArmAndIntakeFunctions {

    private final Gamepad gamepad2;
    private final Arm arm;
    private final Intake intake;

    // Encoder counts corresponding to the rotation angles (60 and 75 degrees)
    private static final int ROTATE_60 = 600; // Example value, adjust based on testing
    private static final int ROTATE_75 = 750; // Example value, adjust based on testing
    private final double ARM_POSITION_TOLERANCE = 15.0; // Tolerance for arm position (in degrees)
    private final double LIFT_POSITION_TOLERANCE = 1.0; // Tolerance for lift position

    private static final double ARM_TICKS_PER_DEGREE = 7.7778;


    public ArmAndIntakeFunctions(Arm arm, Intake intake, Gamepad gamepad2) {
        this.arm = arm;
        this.intake = intake;
        this.gamepad2 = gamepad2;
    }

    // Method to rotate arm to a target angle (between 60 and 75 degrees)
    public void rotateArmToTargetAngle(int targetDegrees) {
        int targetPosition;

        // Ensure the target is between 60 and 75 degrees
        if (targetDegrees >= 60 && targetDegrees <= 75) {
            targetPosition = (int) (ROTATE_60 + ((targetDegrees - 60) / 15.0) * (ROTATE_75 - ROTATE_60));
            arm.rotationMotor.setTargetPosition(targetPosition);
            arm.rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.rotationMotor.setPower(1.0); // Set the motor power
        } else {
            // Target degrees are out of bounds, handle accordingly
            System.out.println("Target degrees out of range (60-75)");
        }
    }

    // Helper method to stop and reset the arm motor encoder
    private void stopAndResetEncoder() {
        arm.rotateArm(0); // Stop arm rotation
        arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset motor encoder
        arm.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Resume normal operation
    }

    // Check if the lift is at the target position within a tolerance
    public boolean isLiftAtTarget(double targetPosition) {
        return Math.abs(arm.getArmEncoderValue() - targetPosition) <= LIFT_POSITION_TOLERANCE;
    }

    // Check if the arm is at the target position within a tolerance
    public boolean isArmAtTarget(double targetPosition) {
        return Math.abs(arm.getRotatedArmPosition() - targetPosition) <= ARM_POSITION_TOLERANCE;
    }

    // Rotate the arm to a straight-up position and reset encoder
    public void armTo90Degrees() {
        arm.rotateArm(90.0 - arm.getRotatedArmPosition()); // Rotate arm directly to 90°
        stopAndResetEncoder();
    }

    // Rotate the arm to the down position (180 degrees) and reset encoder
    public void armToDownPosition() {
        arm.rotateArm(0.0 - arm.getRotatedArmPosition()); // Rotate arm directly to 180°
        stopAndResetEncoder();
    }

    public void collectSample() {
        if (isArmAtTarget(arm.ARM_COLLECT)) {
            arm.moveArmToPosition(arm.ARM_COLLECT);
        } else {
            arm.moveArmToPosition(arm.ARM_COLLECT);
            if (isArmAtTarget(arm.ARM_COLLECT)) {
                intake.collect();
            }
        }
    }
    public void depositGameElement() {
        if (isArmAtTarget(arm.ARM_SCORE_SAMPLE_IN_HIGH)) {
            intake.deposit();
        } else {
            arm.moveArmToPosition(arm.ARM_SCORE_SAMPLE_IN_HIGH); // Move arm to scoring position
            if (isArmAtTarget(arm.ARM_SCORE_SAMPLE_IN_HIGH)) {
                intake.deposit(); // Activate deposit
            }
        }
    }

    public void stopIntake() {
        intake.stop(); // Stop the intake
    }

    public void resetArm() {
        arm.resetArm(); // Reset arm to the initial position
    }

    public void scoreHighBasket() {
        arm.toPoint(-3750); // Move arm to the highest position
        if (arm.getArmEncoderValue() == -3750) {
            rotateArmToTargetAngle(ROTATE_60); // Lower arm
            if (isArmAtTarget(ROTATE_60)) {
                // Wait for the right trigger press to start the outtake
                telemetry.addData("Status", "Press right trigger to start outtake");
                telemetry.update();

                // Check if the right trigger is pressed on gamepad2
                if (gamepad2.right_trigger > 0) {
                    intake.setIntakePower(-1.0); // Outtake for scoring
                    telemetry.addData("Status", "Outtaking...");
                    telemetry.update();
                }
            }
        }
    }

    public void scoreLowBasket() {
        arm.toPoint(-1000); // Move to scoring height
        arm.rotateArm(60); // Fine adjustment
        intake.setIntakePower(-1.0); // Outtake
    }

    // public void scoreSpecimen() {
    //  arm.rotateArm(-0.5); // Fine adjustment
    // intake.setPivotPosition(0.5); // Position intake pivot
    // }

    public void levelTwoAscent() {
        arm.rotateArm(60); // Fine adjustment
        arm.toPoint(-1250); // Ascend to level two
        arm.toPoint(0); // Reset position
    }

    // Check if scoring in the high basket is finished
    public boolean isFinished() {
        double targetPosition = -5000; // High basket scoring position
        double currentArmPosition = arm.getRotatedArmPosition();
        boolean isArmAtPosition = Math.abs(currentArmPosition - targetPosition) < 15; // Adjust tolerance

        boolean isIntakeClosed = Math.abs(intake.getIntakePosition() - 0.0) < 0.1; // Check intake closed

        return isArmAtPosition && isIntakeClosed;
    }
}

