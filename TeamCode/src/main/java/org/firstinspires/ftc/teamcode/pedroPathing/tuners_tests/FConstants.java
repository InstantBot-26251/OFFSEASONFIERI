package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Follower constants for Pedro Pathing
public class FConstants {
    public static final String MOTOR_FL_NAME = "frontLeft";
    public static final String MOTOR_FR_NAME = "frontRight";
    public static final String MOTOR_BL_NAME = "backLeft";
    public static final String MOTOR_BR_NAME = "backRight";
    

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
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        // Robot mass in kg
        FollowerConstants.mass = 9.253;
        // Max velocities
        FollowerConstants.xMovement = (64.97901255690206 + 64.40951129582922 + 64.67623973455956)/3;
        FollowerConstants.yMovement = (52.70229549858514 + 52.078727662094 + 52.54129725178397)/3;
        // Robot deceleration
        FollowerConstants.forwardZeroPowerAcceleration = ((-35.403735512981406) + (-33.29719746540844) + (-34.971722286313934) + (-33.88923247219733) + (-35.727653860349875) + (-35.80014382795648) + (-32.81373258023366))/7;
        FollowerConstants.lateralZeroPowerAcceleration = ((-69.28834524634038) + (-72.68230416299497) + (-72.49874181472346) + (-70.52467494892589))/4;
        // Zero power multiplier
        FollowerConstants.zeroPowerAccelerationMultiplier = 3.75;
        // Whether or not to use dual PID
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.useSecondaryHeadingPID = false;
        // Primary PID coefficients
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.05, 0, 0.00006, 0.6, 0);
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.7, 0, 0.05, 0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(3, 0, 0.15, 0);
        // Centripetal force correction-
        FollowerConstants.centripetalScaling = 0.0007;
    }
}