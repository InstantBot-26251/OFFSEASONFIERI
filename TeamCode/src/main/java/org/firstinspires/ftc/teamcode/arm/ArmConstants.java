
package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;


@Config
public class ArmConstants {
    private static final double PIVOT_DOWN_ENCODER = 0; // Fully pivoted down position
    private static final double PIVOT_UP_ENCODER = 90;       // Fully pivoted up position
    private static final double MAX_ENCODER_EXTENSION = 1000; // Example max encoder value at full extension
    private static final double MIN_ENCODER_EXTENSION = -4000; // Minimum slide retraction
    private static final double SLIDE_AT_42_INCHES = -2764; // Encoder value for 42 inches when pivot is fully down

    public static final double SLIDE_KP = 1.0;
    public static final double SLIDE_KI = 0.0;
    public static final double SLIDE_KD = 0.2;
    public static final double SLIDE_KF = 1;

    public static final double PIVOT_KP = 1.15;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 0.25;
    public static final double PIVOT_KF = 1.1;

    public static final double MAX_VELOCITY = 500;
    public static final double MAX_ACCELERATION = 100;

    public static final double PIVOT_MAX_VELOCITY = 400;
    public static final double PIVOT_MAX_ACCELERATION = 80;

    public static final double ARM_TICKS_PER_DEGREE = 4.67;

    // Slide Constants
    public static final double SLIDE_MIN_POSITION = -4;
    public static final double SLIDE_MAX_POSITION = -2200;
    public static double SLIDE_ERROR_TOLERANCE = 15.0;
    public static double SLIDE_TIMEOUT = 1400;
    public static double ZEROING_TIMEOUT = 2000;
    public static double ZEROING_VELOCITY_ERROR = 5;

    // Positions
    public static double SLIDE_REST_POSITION = 0;
    public static double SLIDE_SAMPLE_COLLECT_POSITION = -1150.0;
    public static double SLIDE_SPECIMEN_COLLECT_POSITION = -800;
    public static double SLIDE_CHAMBER_POSITION = -1507;
    public static double SLIDE_CHAMBER_SCORE_OFFSET = 350;
    public static double SLIDE_BASKET_POSITION = -2000;

    // Constants
    public static double PIVOT_ERROR_TOLERANCE = 5;
    public static double PIVOT_TIMEOUT = 1500;

    // Positions
    public static double PIVOT_REST_POSITION = 0;
    public static double PIVOT_SCORE_SPECIMEN_POSITION = -590;
    public static double PIVOT_SCORE_BASKET_POSITION = -590;
}
