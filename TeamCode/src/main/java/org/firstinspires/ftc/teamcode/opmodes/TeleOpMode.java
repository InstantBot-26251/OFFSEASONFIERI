package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    Intake claw;
    ClawState clawState;


    @Override
    public void init() {
        // Initialize arm and intake systems first
        arm = new Arm2(hardwareMap);  // Initialize arm system
        claw = new Intake(hardwareMap, clawState);  // Initialize intake system

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

        // AUTOMATIC ARM CONTROLS:
        if (gamepad2.dpad_up) {
            //SCORING POSITION FOR HIGH BASKET
            arm.setPivotPosition(-1053);
            if (!arm.isPivotBusy()) {
                arm.setSlidePosition(-2191);
            }
            telemetry.addData("PRESS X TO SCORE", "HIGH BASKET");
        }

        if (gamepad2.dpad_down) {
            //COLLECTING
            if (arm.getPivotPosition() < 259) {
                arm.setSlidePosition(0);
                if (!arm.isSlideBusy()) {
                    arm.setPivotPosition(259); //TODO: CHANGE WITH ACTUAL ENCODER VALUE
                    if (!arm.isPivotBusy()) {
                        arm.setSlidePosition(0); //TODO: CHANGE WITH ACTUAL ENCODER VALUE
                    }
                }
            }
        }
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