package org.firstinspires.ftc.teamcode.chassis;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Fieri;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.util.subsystem.SubsystemTemplate;

public class Chassis extends SubsystemTemplate {
    Follower follower;
    Telemetry telemetry;
    boolean isRobotCentric;
    double m = 1.0;

    private static final Chassis INSTANCE = new Chassis();

    public static Chassis getInstance() {
        return INSTANCE;
    }

    private Chassis() {
        isRobotCentric = false;
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        follower = new Follower(hardwareMap);
        follower.setPose(RobotStatus.robotPose);
    }

    @Override
    public void onTeleopInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        follower = new Follower(hardwareMap);
        follower.setPose(RobotStatus.robotPose);
        follower.startTeleopDrive();
        setMaxPower(1.0);
    }


    public Pose getPoseEstimate() {
        return follower.getPose();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public boolean isRobotCentric() {
        return isRobotCentric;
    }

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public void setPosition(Pose pose) {
        follower.setPose(pose);
    }

    public void setDriveVectors(double fwd, double str, double rot) {
        follower.setTeleOpMovementVectors(fwd * m, str * m, rot * m, isRobotCentric);
    }

    public void resetHeading() {
        Pose oldPose = getPoseEstimate();
        follower.setPose(new Pose(oldPose.getX(), oldPose.getY()));
    }

    public void enableSlowMode() {
        m = 0.30;
    }

    public void disableSlowMode() {
        m = 1.0;
    }


    public void followPath(Path path) {
        follower.followPath(path);
    }

    public void breakFollowing() {
        follower.breakFollowing();
    }

    public void toggleRobotCentric() {
        isRobotCentric = !isRobotCentric;
    }


    @Override
    public void periodic() {
        follower.update();
        RobotStatus.robotPose = follower.getPose();

        telemetry.addData("Robot Centric", isRobotCentric());
        telemetry.addData("Path exists", follower.getCurrentPath() != null);
    }

}