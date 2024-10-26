/*** package org.firstinspires.ftc.teamcode.chassis;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class Chassis extends SubsystemBase {
    Telemetry telemetry;
    Follower follower;
    boolean isRobotCentric;

    private static Chassis INSTANCE = null;

    private Chassis() {
        isRobotCentric = false;
        follower = new Follower(new Pose());
        this.telemetry = RobotCore.getTelemetry();
    }
    public void setPosition(Pose pose) {
        follower.setPose(pose);
    }

    public void resetHeading() {
        Pose oldPose = follower.getPose();
        follower.setPose(new Pose(oldPose.getX(), oldPose.getY()));
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void setTeleopMovementVectors(double fwd, double str, double rot) {
        follower.setTeleOpMovementVectors(fwd, str, rot, true);

    }

    public void followPath(Path path) {
        follower.followPath(path);
    }

    // GETTERS

    public static Chassis getInstance() {
        if (INSTANCE == null) INSTANCE = new Chassis();
        return INSTANCE;
    }

    public Pose getPosition() {
        return follower.getPose();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }
    /// @Override
    public void periodic() {
        follower.update();
    }
}
***/