package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.RobotStatus.Alliance.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Fieri;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;

@Config
@TeleOp(name = "Period of Manual Control")
public class TeleOpMode extends OpMode {
    private final Fieri fieri = Fieri.getInstance();
    private boolean lastSquare = false;

//    private Chassis2 chassis;
//    Arm2 arm;
//    Intake claw;
//    ClawState clawState;
//    Follower follower;
//    private DcMotorEx leftFront;
//    private DcMotorEx leftRear;
//    private DcMotorEx rightFront;
//    private DcMotorEx rightRear;


    @Override
    public void init() {
        fieri.teleopInit(telemetry, hardwareMap, gamepad1, gamepad2);
        lastSquare = gamepad1.square;

        telemetry.addData("Alliance", RobotStatus.alliance);
        telemetry.addData("Status", RobotStatus.robotState);
        telemetry.update();
     /*   // Initialize arm and intake systems first
        arm = new Arm2(hardwareMap);  // Initialize arm system
        claw = new Intake(hardwareMap, clawState);  // Initialize intake system

//        // Initialize chassis
//        chassis = new Chassis2(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize follower
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
*/
    }

    @Override
    public void init_loop() {
        // Allow for manual toggling of alliance
        if (gamepad1.square != lastSquare && gamepad1.square) {
            switch (RobotStatus.alliance) {
                case NONE:
                case RED:
                    RobotStatus.alliance = BLUE;
                    break;
                case BLUE:
                    RobotStatus.alliance = RED;
                    break;
            }
        }
    }
    @Override
    public void loop() {
        fieri.periodic();
//        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//        arm.setPivotPowerAuto();
//        arm.setSlidePowerAuto();
//        // Read inputs for controlling chassis
//        double y = AvyuktResponseCurve(gamepad1.left_stick_y);
//        double x = -AvyuktResponseCurve(gamepad1.left_stick_x);
//        double rx = -AvyuktResponseCurve(gamepad1.right_stick_x);
//        double y2 = IshaanResponseCurve(gamepad2.left_stick_y);
//        double y3 = IshaanResponseCurve(gamepad2.right_stick_y);
//
////        // Drive control
////        chassis.drive(x, y, rx);
////        if (gamepad1.options) {
////            chassis.resetYaw();
////        }
//
//        // AUTOMATIC ARM CONTROLS:
//        if (gamepad2.dpad_up) {
//            if (arm.getSlidePosition() > 0) {
//                arm.setSlidePosition(0);          //  SCORING POSITION FOR HIGH BASKET
//                if (!arm.isSlideBusy()) {
//                    arm.setPivotPosition(-1053);
//                    if (!arm.isPivotBusy()) {
//                        arm.setSlidePosition(-2191);
//                    }
//                    telemetry.addData("PRESS RIGHT TRIGGER TO SCORE", "HIGH BASKET");
//                }
//            }
//            else {
//            arm.setPivotPosition(-1053);
//            if (!arm.isPivotBusy()) {
//                arm.setSlidePosition(-2191);
//            }
//            telemetry.addData("PRESS RIGHT TRIGGER TO SCORE", "HIGH BASKET");
//        }
//        }
//        if (gamepad2.dpad_left) {
//            // SPECIMEN SCORING POSITION
//            arm.setSlidePosition(0);
//            arm.setPivotPosition(0);
//        }
//        if (gamepad2.dpad_down) {
//            //COLLECTING
//            if (arm.getSlidePosition() > 0) {
//                arm.setSlidePosition(0);
//                if(!arm.isSlideBusy()) {
//                if (arm.getPivotPosition() < 259) {
//                    arm.setPivotPosition(259);
//                    if (!arm.isPivotBusy()) {
//                        arm.setPivotPosition(259); //TODO: CHANGE WITH ACTUAL ENCODER VALUE
//                        if (!arm.isPivotBusy()) {
//                            arm.setSlidePosition(0); //TODO: CHANGE WITH ACTUAL ENCODER VALUE
//                            telemetry.addData("PRESS LEFT TRIGGER", "COLLECTION");
//                        }
//                    }
//                }
//                }
//
//            } else {
//                if (arm.getPivotPosition() < 259) {
//                    arm.setSlidePosition(0);
//                    if (!arm.isSlideBusy()) {
//                        arm.setPivotPosition(259); //TODO: CHANGE WITH ACTUAL ENCODER VALUE
//                        if (!arm.isPivotBusy()) {
//                            arm.setSlidePosition(0); //TODO: CHANGE WITH ACTUAL ENCODER VALUE
//                        }
//                    }
//                }
//            }
//
//        }
//        // Arm control with limit checks
//        if (arm.getSlidePosition() <= -2191 && y2 < 0 || arm.getSlidePosition() >= 0 && y2 > 0) {
//            arm.setSlidePower(0); // Stop downward/upward movement
//        }
//        else {
//            arm.setSlidePower(y2);
//        }
//        arm.setPivotPower(y3); // Set the pivot power
//
//        // Claw controls
//        if (gamepad2.right_trigger > 0) {
//            claw.openClaw();
//        }
//
//        if (gamepad2.left_trigger > 0) {
//            claw.closeClaw();
//        }
//        // Wrist controls
//        if (gamepad2.right_bumper) {
//            claw.setWristPower(-1); // Rotate right
//        } else if (gamepad2.left_bumper) {
//            claw.setWristPower(1); // Rotate left
//        } else {
//            claw.setWristPower(0); // Stop rotating when no button is pressed
//        }
//
//        // Telemetry for Debugging
//        telemetry.addData("Slide Input (Left Stick Y):", gamepad2.left_stick_y);
//        telemetry.addData("Pivot Input (Right Stick Y):", gamepad2.right_stick_y);
//        telemetry.addData("Slide Encoder:", arm.getSlidePosition());
//        telemetry.addData("Pivot Encoder:", arm.getPivotPosition());
//        telemetry.addData("Claw Position:", claw.getClawPosition());
//        telemetry.addData("Claw State:", clawState);
//        telemetry.addData("Pivot Busy", arm.isPivotBusy());
//        telemetry.addData("Slide Busy", arm.isSlideBusy());
//        telemetry.update();  // This should be at the end of loop()
    }
    @Override
    public void stop() {
        fieri.disabledInit();
    }
}