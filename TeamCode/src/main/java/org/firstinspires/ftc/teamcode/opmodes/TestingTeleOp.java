package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opmodes.util.AutoState;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.util.ArmAndIntakeFunctions;
import org.firstinspires.ftc.teamcode.util.CollectSample;
import org.firstinspires.ftc.teamcode.util.LevelTwoAscent;
import org.firstinspires.ftc.teamcode.util.ScoreHighBasket;

@TeleOp(name = "TestingTeleOp")
public class TestingTeleOp extends OpMode {
    private Follower follower;

    private Arm arm;
    private Intake intake;
    private ArmAndIntakeFunctions functions;
    private CollectSample collection;
    private LevelTwoAscent ascent;
    private ScoreHighBasket scorehighbasket;
    
    // Flag to track if scoreHighBasket has been initiated
    private final boolean scoreHighBasketInitiated = false;

    @Override
    public void init() {
        // Initialize arm and intake systems first
        arm = new Arm(hardwareMap, 1, 0, 0, 1, 1, 0, 0, 1);  // Initialize arm system
        intake = new Intake(hardwareMap);  // Initialize intake system

        // Initialize functions
        functions = new ArmAndIntakeFunctions(arm, intake, gamepad2);
        collection = new CollectSample(arm, intake, functions);
        scorehighbasket = new ScoreHighBasket(arm, intake, gamepad2, functions);
        ascent = new LevelTwoAscent(arm, intake, functions);

        follower = new Follower(hardwareMap);
        follower.setPose(new Pose());

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // TeleOp movement with response curves
        follower.setTeleOpMovementVectors(
                -applyResponseCurve(gamepad1.left_stick_y),
                applyResponseCurve(gamepad1.left_stick_x),
                applyResponseCurve(gamepad1.right_stick_x),
                true
        );

        // Handle collection with 'a' button
        if (gamepad2.a) {
            collection.drive();
        }

        // Handle scoring in the high basket with the 'b' button
        if (gamepad2.b && !scorehighbasket.isScoringInProgress()) {
            scorehighbasket.execute(); // Initiate scoring high basket

    }

        // Handle level two ascent with 'y' button
        if (gamepad2.y) {
            ascent.drive();
        }

        // Arm positioning with D-pad
        if (gamepad2.dpad_down) {
            functions.armToDownPosition();
        }

        if (gamepad2.dpad_up) {
            functions.armTo90Degrees();
        }

        // Rotation control using gamepad2
        // Note: This is separate from scoreHighBasket's rotation
        if (gamepad2.left_bumper) {
            arm.rotateArm(60);  // Rotate arm clockwise
        } else if (gamepad2.right_bumper) {
            arm.rotateArm(75);  // Rotate arm counterclockwise
        } else {
            arm.stopRotationMotor();  // Stop rotation
        }

        // Intake control using gamepad2
        if (gamepad2.right_trigger > 0.1) {
            intake.setIntakePower(1.0);  // Activate intake
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setIntakePower(-1.0);  // Reverse intake
        } else {
            intake.setIntakePower(0);  // Stop intake
        }

        // Pivot control for intake
        // if (gamepad2.dpad_left) {
            // intake.setPivotPosition(0.0);  // Pivot intake to a certain position
        // } else if (gamepad2.dpad_right) {
        //    intake.setPivotPosition(1.0);  // Pivot intake to another position
        // }

        // Telemetry for diagnostics
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Servo Position", gamepad2.a ? "Open" : gamepad2.b ? "Closed" : "Neutral");
        telemetry.addData("Arm Set Point", arm.getSetPoint());
        telemetry.addData("Lift Encoder Value", arm.getArmEncoderValue());
        telemetry.addData("Rotated Arm Encoder Value", arm.getRotationEncoderValue());
        telemetry.update();
    }

    // Response curve function for finer joystick control
    public double applyResponseCurve(double input) {
        double exponent = 2;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }
}
