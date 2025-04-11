package org.firstinspires.ftc.teamcode.util.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotStatus;

public class AllianceToggler {
    private final Gamepad gamepad1;
    private boolean previousStart = false;
    private final ElapsedTime debounce = new ElapsedTime();
    private final ElapsedTime feedbackTimer = new ElapsedTime();
    private boolean showFeedback = false;
    private String feedbackMessage = "";

    public AllianceToggler(Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
    }
    public void update() {
        boolean x = gamepad1.start;

        if (x && !previousStart && debounce.seconds() > 0.5) {
            toggleAlliance();
            giveAllianceFeedback();
            debounce.reset();
        }

        previousStart = x;
    }

    private void toggleAlliance() {
        switch (RobotStatus.alliance) {
            case RED:
                RobotStatus.alliance = RobotStatus.Alliance.BLUE;
                break;
            case BLUE:
            case NONE:
            default:
                RobotStatus.alliance = RobotStatus.Alliance.RED;
                break;
        }
    }

    public void giveAllianceFeedback() {
        if (showFeedback) {
            telemetry.addLine(feedbackMessage);
            telemetry.update();

            // Only show for 2 seconds
            if (feedbackTimer.seconds() > 5.0) {
                showFeedback = false;
            }
        }
    }
}
