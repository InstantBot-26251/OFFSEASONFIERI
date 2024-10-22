package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Arm2;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.util.ArmAndIntakeFunctions;
import org.firstinspires.ftc.teamcode.util.CollectSample;
import org.firstinspires.ftc.teamcode.util.LevelTwoAscent;
import org.firstinspires.ftc.teamcode.util.ScoreHighBasket;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    public LevelTwoAscent ascent;
    public ScoreHighBasket scorehighbasket;
    public ArmAndIntakeFunctions functions;
    private Follower follower;
    Arm arm;
    Arm2 arm2;
    Intake intake;
    CollectSample collection;


    @Override
    public void init() {
        // Initialize arm and intake systems first
        arm2 = new Arm2(hardwareMap);  // Initialize arm system
        intake = new Intake(hardwareMap);  // Initialize intake system

        // Initialize functions
        functions = new ArmAndIntakeFunctions(arm2, intake, gamepad2);
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
        follower.setTeleOpMovementVectors(-applyResponseCurve(gamepad1.left_stick_y), applyResponseCurve(gamepad1.left_stick_x), applyResponseCurve(gamepad1.right_stick_x), true);

        /*** if (gamepad2.a) {
            collection.drive();
        }

        if (gamepad2.b) {
            scorehighbasket.execute();
        }

        if (gamepad2.y) {
            ascent.drive();
        }

        // if (gamepad2.dpad_down) {
            // functions.armToDownPosition();
        // }

        // if (gamepad2.dpad_up) {
            // functions.armTo90Degrees();
        // }
        ***/
        // Arm control using gamepad2
        if (gamepad2.dpad_down) {
            arm2.toPoint(-10);  // target position
        }

        if (gamepad2.dpad_up) {
            arm2.toPoint(-4250); // Use arm position constant for scoring
        }
        arm2.setPower();

        // Control for the pivot motor using gamepad2
        if (gamepad2.dpad_left) {
            arm2.toPivotPoint(300);  // Set target position for pivot left
        }
        if (gamepad2.dpad_right) {
            arm2.toPivotPoint(-300); // Set target position for pivot right
        }
        arm2.setPivotPower();  // Set power for the pivot motor


        // Intake control using gamepad2
        if (gamepad2.right_trigger > 0.1) {
            intake.setIntakePower(1.0);  // Activate intake
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setIntakePower(-1.0);  // Reverse intake
        } else {
            intake.setIntakePower(0);  // Stop intake
        }


        // Telemetry for diagnostics
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Servo Position", gamepad2.a ? "Open" : gamepad2.b ? "Closed" : "Neutral");
        telemetry.addData("Arm Set Point", arm2.getSetPoint());
        telemetry.addData("Lift Encoder Value", arm2.getEncoderValue());
        telemetry.addData("Rotated Arm Encoder Value", arm2.getPivotEncoderValue());
        telemetry.addData("Ticks", arm2.pivotMotor.getCurrentPosition());
        telemetry.update();
    }

    // Response curve function for finer joystick control
    public double applyResponseCurve(double input) {
        double exponent = 2;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }
}