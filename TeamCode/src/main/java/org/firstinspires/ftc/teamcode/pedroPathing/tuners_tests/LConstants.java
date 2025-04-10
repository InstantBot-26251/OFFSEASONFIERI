package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "otos";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(2.5, 0, Math.toRadians(90));
        OTOSConstants.linearScalar = (1.4802054686921082 + 1.5041696385542167 + 1.4840544427934619)/3;
        OTOSConstants.angularScalar = (0.9343998175000557 + 0.9597844234205234)/2;
    }
}
