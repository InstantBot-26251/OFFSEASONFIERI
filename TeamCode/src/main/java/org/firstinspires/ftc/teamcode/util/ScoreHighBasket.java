package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ScoreHighBasket {
    private final Arm arm;
    private final Intake intake;
    private final Gamepad gamepad;
    private boolean isScoringHighBasket = false;

    // Constructor to initialize arm, intake, and gamepad
    public ScoreHighBasket(Arm arm, Intake intake, Gamepad gamepad) {
        this.arm = arm;
        this.intake = intake;
        this.gamepad = gamepad;
    }

    // Method to score in the high basket
    public void execute() {
        if (!isScoringHighBasket) {
            arm.toPoint(-3750); // Move arm to the highest position
            if (arm.getArmEncoderValue() == -3750) { // Check if the arm has reached the target position
                arm.rotateArm(60); // Lower the arm to 60 degrees for scoring
                isScoringHighBasket = true; // Indicate that scoring is in progress
            }
        } else {
            // Check if the arm has reached the position
            if (arm.getRotatedArmPosition() == Arm.ROTATE_60) {
                // Wait for the right trigger press to start the outtake
                if (gamepad.right_trigger > 0) {
                    intake.setIntakePower(-1.0); // Activate outtake for scoring
                }
            }
        }
    }

    // Method to check if scoring in the high basket is finished
    public boolean isFinished() {
        // Check if the arm is at the target position and intake is stopped
        boolean armAtPosition = arm.getRotatedArmPosition() == Arm.ROTATE_60; // Check if at lowered position
        boolean intakeStopped = intake.getIntakePosition() == 0.0; // Ensure intake is stopped

        if (armAtPosition && intakeStopped) {
            isScoringHighBasket = false; // Reset scoring state
        }

        return armAtPosition && intakeStopped; // Return if scoring is finished
    }

    // Method to check if scoring is currently in progress
    public boolean isScoringInProgress() {
        return isScoringHighBasket;
    }
}