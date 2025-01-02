package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Fieri;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class FollowPathCommand extends CommandBase {
    private final Telemetry telemetry;
    private final Chassis chassis;
    private final Path path;
    private final double maxPower;

    public FollowPathCommand(Path path) {
        this(path, 1.0);
    }

    public FollowPathCommand(Path path, double maxPower) {
        telemetry = Fieri.getInstance().getTelemetry();
        chassis = Chassis.getInstance();
        this.path = path;
        this.maxPower = maxPower;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setMaxPower(maxPower);
        chassis.followPath(path);
    }

    @Override
    public void execute() {
        Pose pose = chassis.getPoseEstimate();
        telemetry.addLine();
        telemetry.addData("Path Running", chassis.isBusy());
        telemetry.addData("Pose", pose.getX() + ", " + pose.getY() + ", " + pose.getHeading());
    }

    @Override
    public boolean isFinished() {
        return !chassis.isBusy();
    }
}

