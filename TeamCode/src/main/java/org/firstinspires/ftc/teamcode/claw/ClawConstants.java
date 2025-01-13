package org.firstinspires.ftc.teamcode.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConstants {
    public static long GRAB_DELAY = 225;

    public static double CLAW_CLOSE_POSITION = 0.4;
    public static double CLAW_OPEN_POSITION = 1.0;


    public static ClawState INIT_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0);
    public static ClawState REST_STATE = new ClawState(CLAW_CLOSE_POSITION, 1.0);

    // SAMPLE POSITIONS
    public static ClawState SAMPLE_COLLECTING_STATE = new ClawState(CLAW_OPEN_POSITION, 1.0);
    public static ClawState SAMPLE_SCORING_STATE = new ClawState(CLAW_CLOSE_POSITION, 0);

    // SPECIMEN POSITIONS
    public static ClawState SPECIMEN_COLLECT_STATE = new ClawState(CLAW_OPEN_POSITION, 1.0);
    public static ClawState SPECIMEN_SCORING_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0);
    public static ClawState SPECIMEN_SCORE_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0);

}