package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.*;

public class RobotCore extends Robot {
    static Telemetry telemetry;
    Arm arm;
    GamepadEx driveController;
    GamepadEx manipController;
    Chassis chassis;

    TeleopDriveCommand driveCommand;

    public enum OpModeType {
        TELEOP,
        AUTO,
        EMPTY
    }

    public static RobotCore INSTANCE = null;

    public static RobotCore getInstance() {
        return INSTANCE;
    }

    public RobotCore(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, OpModeType type) {
        RobotCore.telemetry = telemetry;

        this.driveController = new GamepadEx(gamepad1);
        this.manipController = new GamepadEx(gamepad2);

        initSubsystems();
        setupOpMode(type);
    }

    public void initSubsystems() {
        chassis = new Chassis();
        arm = new Arm();
        claw = new Claw();
        llVision = new LLVision();
        register(chassis, arm, claw, llVision);

        telemetry.addData("Status", "Robot initialized, ready to enable");
        telemetry.update();

        INSTANCE = this;
    }


    public void setDriveControls() {
        driveCommand = new TeleopDriveCommand(
                chassis,
                () -> driveController.getLeftX(),
                () -> driveController.getLeftX(),
                () -> driveController.getRightX()
        );
        chassis.setDefaultCommand(driveCommand);
    }

    public void setupOpmode(OpModeType type) {
        switch (type) {
            case TELEOP:
                chassis.setPosition(RobotGlobal.robotPose);
                chassis.startTeleopDrive();
                setDriveControls();
                Commands.runOnce(() -> setControllerColors(1, 1, 0)).andThen(new InstantCommand(llVision::setYellow));
                break;
            case EMPTY:
                schedule(Commands.none());
                break;
        }
    }
}
