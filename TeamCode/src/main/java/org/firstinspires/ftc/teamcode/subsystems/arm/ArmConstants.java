
package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Arm2;


@Config
public class ArmConstants {
    public static final double ARM_KP = 1.0;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.2;
    public static final double ARM_KF = 1.0;

    public static final double PIVOT_KP = 1.15;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 0.25;
    public static final double PIVOT_KF = 1.1;

    public static final double MAX_VELOCITY = 500;
    public static final double MAX_ACCELERATION = 100;

    public static final double PIVOT_MAX_VELOCITY = 400;
    public static final double PIVOT_MAX_ACCELERATION = 80;

    public static final double ARM_TICKS_PER_DEGREE = 4.67;
    public static final double PIVOT_DOWN_ENCODER = -2764;
    public static final double PIVOT_UP_ENCODER = 0;
}
