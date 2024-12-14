package org.firstinspires.ftc.teamcode.util;

public class ClawConstants {
    public static long COLLECT_DELAY = 30;
    public static long GRAB_DELAY = 400;

    public static ClawState REST_STATE = new ClawState(1.0);

    // SAMPLE POSITIONS

    public static ClawState SAMPLE_COLLECTING_STATE = new ClawState(0.0);
    public static ClawState SAMPLE_SCORING_STATE = new ClawState(1.0);

    // SPECIMEN POSITIONS
    public static ClawState SPECIMEN_COLLECT_STATE = new ClawState(0.0);
    public static ClawState SPECIMEN_SCORING_STATE = new ClawState(1.0);
    public static ClawState SPECIMEN_SCORE_STATE = new ClawState(1.0);
}
