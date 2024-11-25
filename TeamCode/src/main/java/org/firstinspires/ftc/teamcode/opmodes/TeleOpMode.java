package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.chassis.Chassis2;
import org.firstinspires.ftc.teamcode.util.Arm2;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.util.ArmAndIntakeFunctions;
import org.firstinspires.ftc.teamcode.util.CollectSample;
import org.firstinspires.ftc.teamcode.util.LevelTwoAscent;
import org.firstinspires.ftc.teamcode.util.ScoreHighBasket;

@Config
@TeleOp(name = "TeleOp Mode")
public class TeleOpMode extends OpMode {
    //    private Follower follower;
    private Chassis2 chassis;
    Arm2 arm;
    Intake intake;

    @Override
    public void init() {

        // Initialize arm and intake systems first
        arm = new Arm2(hardwareMap, intake);  // Initialize arm system
        intake = new Intake(hardwareMap);  // Initialize intake system

        // Initialize functions
        chassis = new Chassis2(hardwareMap);

//        follower = new Follower(hardwareMap);
//        follower.setPose(new Pose());

//`       DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
//        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
//        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
//        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        follower.startTeleopDrive();

    }

    @Override
    public void loop() {
//        // TeleOp movement with response curves
//        follower.setTeleOpMovementVectors(
//                -applyResponseCurve(gamepad1.left_stick_y),
//                applyResponseCurve(gamepad1.left_stick_x),
//                applyResponseCurve(gamepad1.right_stick_x),
//                true
//        );
        double y = AvyuktResponseCurve(gamepad1.left_stick_y);
        double x = -AvyuktResponseCurve(gamepad1.left_stick_x);
        double rx = -AvyuktResponseCurve(gamepad1.right_stick_x);
        double y2 = IshaanResponseCurve(gamepad2.left_stick_y);
        double y3 = IshaanResponseCurve(gamepad2.right_stick_y);


        chassis.drive(x, y, rx);
        if (gamepad1.options) {
            chassis.resetYaw();
        }

//          follower.update();
          /***
         if (gamepad2.a) {
         collection.drive();
         }
         if (gamepad2.b) {
         scorehighbasket.execute();
         }

         if (gamepad2.y) {
         ascent.drive();
         }
         ***/
        // Pivot control

        if (Math.abs(y3) > 0.1) {
            double currentPivotPosition = arm.getPivotEncoderValue();
            double targetPivotPosition = currentPivotPosition + (y3 * 10); // Scale joystick input
            arm.toPivotPoint(targetPivotPosition);
        }

        // Slide control with dynamic limits

        if (Math.abs(y2) > 0.1) {
            double currentSlidePosition = arm.getEncoderValue();
            double targetSlidePosition = currentSlidePosition + (y2 * 10);
            double maxSlideExtension = arm.getMaxSlidePosition();

            // Restrict slide extension
            if ((currentSlidePosition >= maxSlideExtension && y2 > 0) ||
                    (currentSlidePosition <= arm.getMinSlidePosition() && y2 < 0)) {
                arm.toPoint(currentSlidePosition); // Maintain position if limits are reached
            } else {
                arm.toPoint(targetSlidePosition);
            }
        }


        // Intake control using gamepad2
        if (gamepad2.right_trigger > 0.1) {
            intake.setIntakePower(1.0);  // Activate intake
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setIntakePower(-1.0);  // Reverse intake
        } else {
            intake.setIntakePower(0);
        }

        arm.update();
        // Telemetry for diagnostics
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Servo Position", gamepad2.a ? "Open" : gamepad2.b ? "Closed" : "Neutral");
        telemetry.addData("Slide Set Point", arm.getSetPoint());
        telemetry.addData("Pivot Set Point", arm.getPivotSetPoint());
        telemetry.addData("Lift Encoder Value", arm.getEncoderValue());
        telemetry.addData("Rotated Arm Encoder Value", arm.getPivotEncoderValue());
        telemetry.addData("Pivot Motor Ticks", arm.pivotMotor.getCurrentPosition());
        telemetry.addData("Y2", y2);
        telemetry.update();
    }


    /**
     * Calculates the maximum allowable slide extension based on the pivot position.
     * @param pivotEncoderValue The current encoder value of the pivot.
     * @return The maximum encoder value the slide can extend to.
     */
    private double calculateMaxSlideExtension(double pivotEncoderValue) {
        final double PIVOT_DOWN_ENCODER = -2764; // Pivot down encoder value
        final double PIVOT_UP_ENCODER = 0;       // Pivot up encoder value
        final double MAX_EXTENSION_ENCODER = arm.getMaxSlidePosition(); // Full extension encoder value

        // At pivot fully down, max extension is -2764
        if (pivotEncoderValue <= PIVOT_DOWN_ENCODER) {
            return -2764;
        }

        // At pivot fully up, max extension is restricted by physical max
        if (pivotEncoderValue >= PIVOT_UP_ENCODER) {
            return MAX_EXTENSION_ENCODER;
        }

        // Linearly interpolate max extension between pivot down and pivot up
        return -2764 + (pivotEncoderValue - PIVOT_DOWN_ENCODER) /
                (PIVOT_UP_ENCODER - PIVOT_DOWN_ENCODER) *
                (MAX_EXTENSION_ENCODER - (-2764));
    }

    // Response curve function for finer joystick control
    public double AvyuktResponseCurve(double input) {
        double exponent = 1.5;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    // Response curve function for finer arm joystick control
    public double IshaanResponseCurve(double input) {
        double exponent = 1.111111;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

}