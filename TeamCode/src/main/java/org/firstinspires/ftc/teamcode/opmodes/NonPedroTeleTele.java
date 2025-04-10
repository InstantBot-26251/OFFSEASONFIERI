package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.FConstants.MOTOR_BL_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.FConstants.MOTOR_BR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.FConstants.MOTOR_FR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.FConstants.MOTOR_FL_NAME;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.chassis.Chassis2;
import org.firstinspires.ftc.teamcode.util.nonftclib.Arm2;
import org.firstinspires.ftc.teamcode.claw.ClawState;
import org.firstinspires.ftc.teamcode.util.nonftclib.Intake;

@Config
@TeleOp(name = "Period of Manual Control without pedrox")
public class NonPedroTeleTele extends OpMode {
    private Chassis2 chassis;
    Arm2 arm;
    Intake claw;
    ClawState clawState;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;


    @Override
    public void init() {
        // Initialize arm and intake systems first
        arm = new Arm2(hardwareMap);  // Initialize arm system
        claw = new Intake(hardwareMap, clawState);  // Initialize intake system


    chassis = new Chassis2(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, MOTOR_FL_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, MOTOR_FR_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, MOTOR_BR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, MOTOR_BL_NAME);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        // Read inputs for controlling chassis
        double y2 = IshaanResponseCurve(gamepad2.left_stick_y);
        double y3 = IshaanResponseCurve(gamepad2.right_stick_y);

        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        // Arm control with limit checks
        if (arm.getSlidePosition() <= -2191 && y2 < 0 || arm.getSlidePosition() >= 0 && y2 > 0) {
            arm.setSlidePower(0); // Stop downward/upward movement
        }
        else {
            arm.setSlidePower(y2);
        }
        arm.setPivotPower(y3); // Set the pivot power

        // Claw controls
        if (gamepad2.right_trigger > 0) {
            claw.openClaw();
        }

        if (gamepad2.left_trigger > 0) {
            claw.closeClaw();
        }
        // Wrist controls
        if (gamepad2.right_bumper) {
            claw.setWristPower(-1); // Rotate right
        } else if (gamepad2.left_bumper) {
            claw.setWristPower(1); // Rotate left
        } else {
            claw.setWristPower(0); // Stop rotating when no button is pressed
        }

        // Telemetry for Debugging
        telemetry.addData("Slide Input (Left Stick Y):", gamepad2.left_stick_y);
        telemetry.addData("Pivot Input (Right Stick Y):", gamepad2.right_stick_y);
        telemetry.addData("Slide Encoder:", arm.getSlidePosition());
        telemetry.addData("Pivot Encoder:", arm.getPivotPosition());
        telemetry.addData("Claw Position:", claw.getClawPosition());
        telemetry.addData("Claw State:", clawState);
        telemetry.addData("Pivot Busy", arm.isPivotBusy());
        telemetry.addData("Slide Busy", arm.isSlideBusy());
        telemetry.update();  // This should be at the end of loop()
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