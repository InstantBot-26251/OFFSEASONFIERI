package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis2;
import org.firstinspires.ftc.teamcode.util.Arm2;
import org.firstinspires.ftc.teamcode.util.ClawState;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Period of Manual Control")
public class TeleOpMode extends OpMode {
    RobotCore robot;
    //    private Follower follower;
    private Chassis2 chassis;
    Arm2 arm;
    Intake intake;
    RobotGlobal robotGlobal;
    ClawState clawState;
    @Override
    public void init() {

        // Initialize arm and intake systems first
        arm = new Arm2(hardwareMap);  // Initialize arm system
        intake = new Intake(hardwareMap, clawState);  // Initialize intake system

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
        if (robotGlobal.getAllianceColor().equals("Red")) {
            gamepad1.setLedColor(250, 0, 0, -1);
            gamepad2.setLedColor(250, 0, 0, -1);
        }

        else if (robotGlobal.getAllianceColor().equals("Blue")) {
            gamepad1.setLedColor(0, 0, 250, -1);
            gamepad2.setLedColor(0, 0, 250, -1);
        }

        else if (robotGlobal.getAllianceColor().equals("None")) {
            gamepad1.setLedColor(255, 255, 0,-1);
            gamepad2.setLedColor(255, 255, 0,-1);
        }

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

        arm.setPower(y2);
        arm.setPivotPower(y3);

        chassis.drive(x, y, rx);
        if (gamepad1.options) {
            chassis.resetYaw();
        }

        if (gamepad2.x) {
            intake.openClaw();
        }

        if (gamepad2.b) {
            intake.closeClaw();
        }

        while (gamepad2.right_bumper) {
            double wristPos = 0.0;
            wristPos += 1;
            intake.setWrist(wristPos);
        }

        while (gamepad2.left_bumper) {
            double wristPos = 0.0;
            wristPos += 1;
            intake.setWrist(wristPos);
        }
//         follower.update();

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