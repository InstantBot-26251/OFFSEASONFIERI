package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    private static final double LINEAR_SCALAR = 46.4331;
    private static final double ANGULAR_SCALAR = 1.2098;

    static {
        // Whether or not to use the "corrected" otos driver
        OTOSConstants.useCorrectedOTOSClass = false;
        // Device name
        OTOSConstants.hardwareMapName = "otos";
        // Localizer units
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        // OTOS position on the robot
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(2.5,0, Math.toRadians(270));
        // Scalars for the sensor
        OTOSConstants.linearScalar = LINEAR_SCALAR;
        OTOSConstants.angularScalar = ANGULAR_SCALAR;
    }

}
