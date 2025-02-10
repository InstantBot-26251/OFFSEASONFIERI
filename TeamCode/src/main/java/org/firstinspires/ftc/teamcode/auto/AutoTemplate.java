package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.Fieri;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import static org.firstinspires.ftc.teamcode.robot.RobotStatus.Alliance.*;

public abstract class AutoTemplate extends LinearOpMode {
    protected final Fieri robot = Fieri.getInstance();
    boolean lastUp;
    boolean lastDown;
    boolean lastStart;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotStatus.resetValues();
        robot.autonomousInit(telemetry, hardwareMap);

        while (opModeInInit() && !gamepad1.options) {
            config();
        }
        sleep(500);

        // Setup poses and paths
        RobotStatus.robotPose = getStartingPose();
        sleep(100);
        buildPaths();

        Chassis.getInstance().setPosition(getStartingPose());

        telemetry.addData("Status", "Scheduling commands");
        telemetry.update();
        if (RobotStatus.alliance == NONE) RobotStatus.alliance = RED;
        Commands.waitUntil(RobotStatus::isEnabled)
                .andThen(Commands.waitMillis(RobotStatus.delayMs))
                .andThen(makeAutoSequence())
                .schedule();

        while (opModeInInit()) {
            telemetry.addData("Status", "Initialized, Ready to start");
            telemetry.addData("Selected auto delay", RobotStatus.delayMs);
            telemetry.addData("Selected alliance", RobotStatus.alliance);
            telemetry.update();

            sleep(50);
        }

        if (RobotStatus.alliance == NONE) RobotStatus.alliance = RobotStatus.Alliance.RED;

        setAutoRan();

        while (opModeIsActive() && !isStopRequested()) {
            robot.periodic();
        }

        robot.disabledInit();
    }

    public void config() {
        // Adjust delay
        if (checkInputs(gamepad1.dpad_up, lastUp)) RobotStatus.delayMs += 100;
        if (checkInputs(gamepad1.dpad_down, lastDown) && RobotStatus.delayMs > 0) RobotStatus.delayMs -= 100;
        // Select alliance
        if (checkInputs(gamepad1.square, lastStart)) {
            switch(RobotStatus.alliance) {
                case NONE:
                case RED:
                    RobotStatus.alliance = BLUE;
                    break;
                case BLUE:
                    RobotStatus.alliance = RED;
                    break;
            }
        }
        // Set old inputs
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastStart = gamepad1.start;

        telemetry.addData("Status", "Configuring Autonomous");
        telemetry.addData("Controls", "\nDelay: UP & DOWN \nSelect alliance: START");
        telemetry.addLine();
        telemetry.addData("Selected auto delay", RobotStatus.delayMs);
        telemetry.addData("Selected alliance", RobotStatus.alliance);
        telemetry.update();
    }

    public boolean checkInputs(boolean current, boolean last) {
        return (last != current) && current;
    }

    protected abstract Pose getStartingPose();

    protected abstract void buildPaths();

    protected abstract Command makeAutoSequence();

    protected abstract void setAutoRan();
}