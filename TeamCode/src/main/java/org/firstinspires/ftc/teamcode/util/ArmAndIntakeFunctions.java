package org.firstinspires.ftc.teamcode.util;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ArmAndIntakeFunctions {

    private final Gamepad gamepad2;
    private final Arm arm;
    private final Arm2 arm2;
    private final Intake intake;

    // Encoder counts corresponding to the rotation angles (60 and 75 degrees)
    private static final int ROTATE_60 = 600; // Example value, adjust based on testing
    private static final int ROTATE_75 = 750; // Example value, adjust based on testing
    private final double ARM_POSITION_TOLERANCE = 15.0; // Tolerance for arm position (in degrees)
    private final double LIFT_POSITION_TOLERANCE = 1.0; // Tolerance for lift position

    private static final double ARM_TICKS_PER_DEGREE = 7.7778;


    public ArmAndIntakeFunctions(Arm2 arm2, Intake intake, Gamepad gamepad2, Arm arm) {
        this.arm2 = arm2;
        this.intake = intake;
        this.gamepad2 = gamepad2;
        this.arm = arm;
    }
    // Check if the lift is at the target position within a tolerance
    public boolean isLiftAtTarget(double targetPosition) {
        return Math.abs(arm2.getEncoderValue() - targetPosition) <= LIFT_POSITION_TOLERANCE;
    }

    // Check if the arm is at the target position within a tolerance
    public boolean isArmAtTarget(double targetPosition) {
        return Math.abs(arm2.getPivotEncoderValue() - targetPosition) <= ARM_POSITION_TOLERANCE;
    }

    // Rotate the arm to a straight-up position and reset encoder
    /*** public void armTo90Degrees() {
        arm2.toPivotPoint(90.0 - arm2.toPivotPoint(300); // Rotate arm directly to 90°
        stopAndResetEncoder();
    }

    // Rotate the arm to the down position (180 degrees) and reset encoder
    public void armToDownPosition() {
        arm.rotateArm(0.0 - arm.getRotatedArmPosition()); // Rotate arm directly to 180°
        stopAndResetEncoder();
    }
    ***/
    public void collectSample() {
        if (isArmAtTarget(arm.ARM_COLLECT)) {
            arm2.toPivotPoint(arm.ARM_COLLECT);
        } else {
            arm2.toPivotPoint(arm.ARM_COLLECT);
            if (isArmAtTarget(arm.ARM_COLLECT)) {
                intake.collect();
            }
        }
    }

    public void depositGameElement() {
        if (isArmAtTarget(arm.ARM_SCORE_SAMPLE_IN_HIGH)) {
            intake.deposit();
        } else {
            arm2.toPivotPoint(arm.ARM_SCORE_SAMPLE_IN_HIGH); // Move arm to scoring position
            if (isArmAtTarget(arm.ARM_SCORE_SAMPLE_IN_HIGH)) {
                intake.deposit(); // Activate deposit
            }
        }
    }

    public void stopIntake() {
        intake.stop(); // Stop the intake
    }

    public void scoreHighBasket() {
        arm2.toPoint(-3750); // Move arm to the highest position
        if (arm2.getEncoderValue() == -3750) {
            arm2.toPivotPoint(ROTATE_60); // Lower arm
            if (isArmAtTarget(ROTATE_60)) {
                // Wait for the right trigger press to start the outtake
                telemetry.addData("Status", "Press right trigger to start outtake");
                telemetry.update();

                // Check if the right trigger is pressed on gamepad2
                if (gamepad2.right_trigger > 0) {
                    intake.deposit(); // Outtake for scoring
                    telemetry.addData("Status", "Outtaking...");
                    telemetry.update();
                }
            }
        }
    }

    public void scoreLowBasket() {
        arm2.toPoint(-1000); // Move to scoring height
        arm2.toPivotPoint(60); // Fine adjustment
        intake.setIntakePower(-1.0); // Outtake
    }

    // public void scoreSpecimen() {
    //  arm.rotateArm(-0.5); // Fine adjustment
    // intake.setPivotPosition(0.5); // Position intake pivot
    // }

    public void levelTwoAscent() {
        arm2.toPivotPoint(60); // Fine adjustment
        arm2.toPoint(-1250); // Ascend to level two
        arm2.toPoint(0); // Reset position
    }

    // Check if scoring in the high basket is finished
    public boolean isFinished() {
        double targetPosition = -5000; // High basket scoring position
        double currentArmPosition = arm2.getPivotEncoderValue();
        boolean isArmAtPosition = Math.abs(currentArmPosition - targetPosition) < 15; // Adjust tolerance

        boolean isIntakeClosed = Math.abs(intake.getIntakePosition() - 0.0) < 0.1; // Check intake closed

        return isArmAtPosition && isIntakeClosed;
    }
}
