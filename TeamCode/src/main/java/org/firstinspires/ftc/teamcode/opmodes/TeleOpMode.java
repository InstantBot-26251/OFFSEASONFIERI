package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

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
    public ArmAndIntakeFunctions functions;
//    private Follower follower;
    private Chassis2 chassis;
    Arm2 arm;
    Intake intake;
    CollectSample collection;
    double x, y, rx;

    @Override
    public void init() {

        // Initialize arm and intake systems first
        arm = new Arm2(hardwareMap);  // Initialize arm system
        intake = new Intake(hardwareMap);  // Initialize intake system

        // Initialize functions
        functions = new ArmAndIntakeFunctions(arm, intake, gamepad2);
        collection = new CollectSample(arm, intake, functions);
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
//        follower.st   `108artTeleopDrive();

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
        y = applyResponseCurve(gamepad1.left_stick_y);
        x = -applyResponseCurve(gamepad1.left_stick_x);
        rx = -applyResponseCurve(gamepad1.right_stick_x);
        double y2 = applyArmResponseCurve(gamepad2.left_stick_y);
        double y3 = applyArmResponseCurve(gamepad2.right_stick_y);


        chassis.drive(x, y, rx);
        if (gamepad1.options) {
            chassis.resetYaw();
        }

//          follower.update();
//           follower.setTeleOpMovementVectors(-applyResponseCurve(gamepad1.left_stick_y), applyResponseCurve(gamepad1.left_stick_x), applyResponseCurve(gamepad1.right_stick_x), true);
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

        arm.setPivotPower(y3);


        // Arm control with an upper limit check
        if (arm.getEncoderValue() <= -2200 && y2 < 0) {
            arm.setPower(0);  // Stop extending if limit is reached and y2 requests upward movement
        } else {
            arm.setPower(y2);  // Allow normal operation, including downward movement
        }
        if (arm.getEncoderValue() > 0 && y2 > 0) {
            arm.setPower(0);
        } else {
            arm.setPower(y2);
        }

        // Intake control using gamepad2
        if (gamepad2.right_trigger > 0.1) {
            intake.setIntakePower(1.0);  // Activate intake
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setIntakePower(-1.0);  // Reverse intake
        } else {
            intake.setIntakePower(0);
        }
        // Telemetry for diagnostics
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Servo Position", gamepad2.a ? "Open" : gamepad2.b ? "Closed" : "Neutral");
        telemetry.addData("Arm Set Point", arm.getSetPoint());
        telemetry.addData("Lift Encoder Value", arm.getEncoderValue());
        telemetry.addData("Rotated Arm Encoder Value", arm.getPivotEncoderValue());
        telemetry.addData("Ticks", arm.pivotMotor.getCurrentPosition());
        telemetry.addData("Y2", y2);
        telemetry.update();
    }

    // Response curve function for finer joystick control
    public double applyResponseCurve(double input) {
        double exponent = 1.275;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    // Response curve function for finer arm joystick control
    public double applyArmResponseCurve(double input) {
        double exponent = 1.275;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

}