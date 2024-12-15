package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis2;
import org.firstinspires.ftc.teamcode.util.Arm2;
import org.firstinspires.ftc.teamcode.util.ClawState;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@Config
@TeleOp(name = "Period of Manual Control")
public class TeleOpMode extends OpMode {
    private Chassis2 chassis;
    Arm2 arm;
    Intake intake;
    ClawState clawState;

    @Override
    public void init() {
        // Initialize arm and intake systems first
        arm = new Arm2(hardwareMap);  // Initialize arm system
        intake = new Intake(hardwareMap, clawState);  // Initialize intake system

        // Initialize functions
        chassis = new Chassis2(hardwareMap);

        // Any setup logic or hardware initialization should go here
        // Avoid any infinite or blocking loops inside init()
    }

    @Override
    public void loop() {
        // Read inputs for controlling chassis
        double y = AvyuktResponseCurve(gamepad1.left_stick_y);
        double x = -AvyuktResponseCurve(gamepad1.left_stick_x);
        double rx = -AvyuktResponseCurve(gamepad1.right_stick_x);
        double y2 = IshaanResponseCurve(gamepad2.left_stick_y);
        double y3 = IshaanResponseCurve(gamepad2.right_stick_y);

        // Drive control
        chassis.drive(x, y, rx);
        if (gamepad1.options) {
            chassis.resetYaw();
        }
        // Arm control with an upper limit check
        if (arm.getSlidePosition() <= -2200 && y2 < 0 && arm.getPivotPosition() < -1053) {
            arm.setSlidePower(0);  // Stop extending if limit is reached and y2 requests upward movement
        } else {
            arm.setSlidePower(y2);  // Allow normal operation, including downward movement
        }


    arm.updatePivot();
    arm.updateSlide();
    arm.setPivotPower(y3); // Set the pivot power

        // Intake controls
        if (gamepad2.right_trigger > 0.1) {
            arm.intake.setIntakePower(1.0);
        } else if (gamepad2.left_trigger > 0.1) {
            arm.intake.setIntakePower(-1.0);
        } else {
            arm.intake.setIntakePower(0);
        }

        // Claw controls
        if (gamepad2.x) {
            arm.intake.openClaw();
        }
        if (gamepad2.b) {
            arm.intake.closeClaw();
        }

        // Telemetry for Debugging
        telemetry.addData("Slide Input (Left Stick Y):", gamepad2.left_stick_y);
        telemetry.addData("Pivot Input (Right Stick Y):", gamepad2.right_stick_y);
        telemetry.addData("Slide Encoder:", arm.getSlidePosition());
        telemetry.addData("Pivot Encoder:", arm.getPivotPosition());
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