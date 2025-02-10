package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class FConstants {
    public static String MOTOR_FL_NAME = "frontLeft";
    public static String MOTOR_BL_NAME = "backLeft";
    public static String MOTOR_FR_NAME = "frontRight";
    public static String MOTOR_BR_NAME = "backRight";

    static {
        // General
        // Whether or not to brake in teleop
        FollowerConstants.useBrakeModeInTeleOp = true;
        // Threshold to cache motor powers
        FollowerConstants.motorCachingThreshold = 0.01;

        // Localizer type
        FollowerConstants.localizers = Localizers.OTOS;
        // Motor names
        FollowerConstants.leftFrontMotorName = MOTOR_FL_NAME;
        FollowerConstants.rightFrontMotorName = MOTOR_FR_NAME;
        FollowerConstants.leftRearMotorName = MOTOR_BL_NAME;
        FollowerConstants.rightRearMotorName = MOTOR_BR_NAME;
        // Motor directions
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        // Robot mass in kg
        FollowerConstants.mass = 9.253;
        // Max velocities
        FollowerConstants.xMovement = 55.78209160367868;
        FollowerConstants.yMovement = 44.222013984179235;
        // Robot deceleration
        FollowerConstants.forwardZeroPowerAcceleration = ((-48.911757201461356 + -47.22430720104032)/2);
        FollowerConstants.lateralZeroPowerAcceleration = ((-81.52315809473562 + -73.50785739800882 + -93.53992554135772 + -95.47662023928717)/4);
        // Zero power multiplier
        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        // Whether or not to use dual PID
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.useSecondaryHeadingPID = false;
        // Primary PID coefficients
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.025, 0, 0.00001, 0.6, 0);
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.7, 0, 0.06, 0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(3.0, 0, 0.15, 0);
        // Secondary PID coefficients
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.015, 0, 0.000005, 0.6, 0);
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.3, 0, 0.01, 0);
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.08, 0);
        // Centripetal force correction
        FollowerConstants.centripetalScaling = 0.0007;
    }

}
