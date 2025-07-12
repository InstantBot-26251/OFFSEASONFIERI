package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.LConstants;

@Config
@Autonomous(name = "Specimen Auto ")
public class SpecAuto extends OpMode {
    private Telemetry telemetryA;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Follower follower;
    private PathChain Path;
    private PathBuilder Park =  new PathBuilder();;
    private AutoConstants AC;
    private final Pose startPose = new Pose(0, 90, Math.toRadians(0));
    private final Pose collectPose = new Pose(AC.WALL_COLLECT_X_POSITION, AC.WALL_COLLECT_Y_POSITION, Math.toRadians(180));
    private final Pose scorePose = new Pose(AC.CHAMBER_X_POSITION, AC.CHAMBER_RIGHT_Y_POSITION, Math.toRadians(0));
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    public void buildPaths() {
        Path = follower.pathBuilder()
                // Line 1 - BezierCurve
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(9.000, 100.000, Point.CARTESIAN),
                        new Point(collectPose)))
                .setTangentHeadingInterpolation()

                // Line 2 - BezierCurve
                .addPath(new BezierCurve(
                        new Point(collectPose),
                        new Point(20.860, 83.664, Point.CARTESIAN),
                        new Point(28.486, 76.710, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0))

                // Line 3 - BezierLine
                .addPath(new BezierLine(
                        new Point(28.486, 76.710, Point.CARTESIAN),
                        new Point(19, 35, Point.CARTESIAN)))
                .setTangentHeadingInterpolation()

                // Line 4 - BezierLine
                .addPath(new BezierLine(
                        new Point(15.925, 17.495, Point.CARTESIAN),
                        new Point(28.486, 76.710, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0.0))

                // Line 5 - BezierLine
                .addPath(new BezierLine(
                        new Point(28.486, 76.710, Point.CARTESIAN),
                        new Point(16.150, 17.271, Point.CARTESIAN)))
                .setTangentHeadingInterpolation()

                // Line 6 - BezierLine
                .addPath(new BezierLine(
                        new Point(15.925, 17.495, Point.CARTESIAN),
                        new Point(28.486, 76.710, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0.0))

                // Line 7 - BezierLine
                .addPath(new BezierLine(
                        new Point(28.486, 76.710, Point.CARTESIAN),
                        new Point(16.150, 17.271, Point.CARTESIAN)))
                .setTangentHeadingInterpolation()

                // Line 8 - BezierLine
                .addPath(new BezierLine(
                        new Point(15.925, 17.495, Point.CARTESIAN),
                        new Point(28.486, 76.710, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0.0))
                .build();
        Park
                .addPath(new BezierLine(
                                new Point(28.486, 76.710, Point.CARTESIAN),
                                new Point(13.682, 14.579, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation();

    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(Park.build(),true);
                    setPathState(2);
                }
        }


    }
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
