package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ScoreHighBasket {
    private final ArmAndIntakeFunctions functions;
    private final Arm arm;
    private final Intake intake;
    private final Gamepad gamepad;
    private boolean isScoringHighBasket = false;

    public ScoreHighBasket(Arm arm, Intake intake, Gamepad gamepad, ArmAndIntakeFunctions functions) {
        this.arm = arm;
        this.intake = intake;
        this.gamepad = gamepad;
        this.functions = functions;
    }

    // Method to score in the high basket
    public void execute() {
        if (!isScoringHighBasket) {
            arm.toPoint(-3750); // Move to the position for scoring
            if (functions.isLiftAtTarget(-3750)) {
                intake.setIntakePower(-1.0); // Activate outtake for scoring
                isScoringHighBasket = true; // Indicate that scoring is in progress
            }
        }
    }

    // Method to check if scoring in the high basket is finished
    public boolean isFinished() {
        // Check if the arm is at the target position and intake is stopped
        boolean armAtPosition = functions.isLiftAtTarget(-3750);
        boolean intakeStopped = intake.getIntakePosition() == 0; // Ensure intake is stopped

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
