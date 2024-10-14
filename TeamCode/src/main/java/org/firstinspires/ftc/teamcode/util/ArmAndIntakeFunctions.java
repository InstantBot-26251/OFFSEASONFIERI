package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * This is the Arm and Intake Functions class. It contains all the arm and intake functions.
 *
 * @author Lakshya Khandelwal - 26251 Instantbots
 * @version 1.3, 10/4/2024
 */
public class ArmAndIntakeFunctions {
    private final Gamepad gamepad2;
    private final Arm arm;
    private final Intake intake;
    private final double ARM_POSITION_TOLERANCE = 2.0; // Tolerance for arm position (in degrees)
    private final double LIFT_POSITION_TOLERANCE = 1.0; // Tolerance for lift position

    public ArmAndIntakeFunctions(Arm arm, Intake intake, Gamepad gamepad2) {
        this.arm = arm;
        this.intake = intake;
        this.gamepad2 = gamepad2;
    }

    // Helper method to stop and reset the arm motor encoder
    private void stopAndResetEncoder() {
        arm.rotateArm(0); // Stop arm rotation
        arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset motor encoder
        arm.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Resume normal operation
    }

    // Check if the lift is at the target position within a tolerance
    public boolean isLiftAtTarget(double targetPosition) {
        return Math.abs(arm.getEncoderValue() - targetPosition) <= LIFT_POSITION_TOLERANCE;
    }

    // Check if the arm is at the target position within a tolerance
    public boolean isArmAtTarget(double targetPosition) {
        return Math.abs(arm.getRotatedArmPosition() - targetPosition) <= ARM_POSITION_TOLERANCE;
    }

    // Rotate the arm to a straight-up position and reset encoder
    public void armTo90Degrees() {
        arm.rotateArm(90 - arm.getRotatedArmPosition()); // Rotate arm directly to 90°
        stopAndResetEncoder();
    }

    // Rotate the arm to the down position (180 degrees) and reset encoder
    public void armToDownPosition() {
        arm.rotateArm(180 - arm.getRotatedArmPosition()); // Rotate arm directly to 180°
        stopAndResetEncoder();
    }

    public void collectSample() {
        armToDownPosition(); // Move the arm to the down position

        if (isArmAtTarget(180)) {
            intake.setIntakePower(1.0); // Start the intake

            // Continuously check for the 'Right Bumper' press on gamepad2 to stop the intake
            while (!gamepad2.right_bumper) {
                telemetry.addData("Status", "Collecting sample... Press Right Bumper to stop.");
                telemetry.update();
            }

            // Stop the intake when 'B' button on gamepad2 is pressed
            intake.setIntakePower(0.0); // Stop intake
        }
    }

    public void scoreHighBasket() {
        armTo90Degrees(); // Move arm to 90° for scoring
        if (isArmAtTarget(90)) {
            arm.toPoint(-5000); // Lower arm
            intake.setIntakePower(-1.0); // Outtake for scoring
        }
    }

    public void scoreLowBasket() {
        arm.toPoint(-1000); // Move to scoring height
        arm.rotateArm(-0.5); // Fine adjustment
        intake.setIntakePower(-1.0); // Outtake
    }

    public void scoreSpecimen() {
        arm.rotateArm(-0.5); // Fine adjustment
        intake.setPivotPosition(0.5); // Position intake pivot
    }

    public void levelTwoAscent() {
        arm.rotateArm(-0.5); // Fine adjustment
        arm.toPoint(-1250); // Ascend to level two
        arm.toPoint(0); // Reset position
    }

    // Check if scoring in the high basket is finished
    public boolean isFinished() {
        double targetPosition = -5000; // High basket scoring position
        double currentArmPosition = arm.getRotatedArmPosition();
        boolean isArmAtPosition = Math.abs(currentArmPosition - targetPosition) < 50; // Adjust tolerance

        boolean isIntakeClosed = Math.abs(intake.getIntakePosition() - 0.0) < 0.1; // Check intake closed

        return isArmAtPosition && isIntakeClosed;
    }
}
