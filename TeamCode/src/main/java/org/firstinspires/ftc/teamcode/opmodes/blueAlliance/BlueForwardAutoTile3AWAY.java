//package org.firstinspires.ftc.teamcode.opmodes.blueAlliance;
//
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.checkAlliance;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.opmodes.util.AutoState;
//import org.firstinspires.ftc.teamcode.auto.*;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.util.Arm2;
//import org.firstinspires.ftc.teamcode.util.ArmAndIntakeFunctions;
//import org.firstinspires.ftc.teamcode.util.Arm;
//import org.firstinspires.ftc.teamcode.util.Intake;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//
//@Autonomous(name = "Blue Alliance Auto Tile 3 Away")
//public class BlueForwardAutoTile3AWAY extends OpMode {
//
//    Pose sampleOnePose = new Pose(65, 120, Math.toRadians(0));
//    Pose scoreOnePose = new Pose(12, 120, 0);
//    Pose sampleTwoPose = new Pose(62, 130, 0);
//    Pose scoreTwoPose = new Pose(18, 130, 0);
//    Pose sampleThreePose = new Pose(62, 134, 0);
//    Pose scoreThreePose = new Pose(22, 134, 0);
//    Pose parkPosition;
//
//    Path preloadToSampleOnePath;
//    Path scoreOnePath;
//    Path sampleTwoPath;
//    Path scoreTwoPath;
//    Path sampleThreePath;
//    Path scoreThreePath;
//    Path scoreToParkPath;
//
//    private Follower follower;
//
//    private AutoState currentState = AutoState.MOVE_TO_FIRST_NEUTRAL_SAMPLE;
//    private long startTime;
//
//    @Override
//    public void init() {
//        PathChain observationPath = new PathBuilder()
//                .addPath(new BezierCurve(
//                        new Point(46, 121.25, Point.CARTESIAN),  // Last sample position as start
//                        new Point(30, 80, Point.CARTESIAN),      // Control point
//                        new Point(14, 14, Point.CARTESIAN)       // Observation zone
//                ))
//                .build();
//
//        follower = new Follower(hardwareMap);
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void init_loop() {
//        telemetry.addData("State", currentState);
//        telemetry.addData("Time", System.currentTimeMillis() - startTime);
//    }
//
//    @Override
//    public void start() {
//        startTime = System.currentTimeMillis();
//        follower.setStartingPose(new Pose(0, 60, 0)); // Starting position
//        follower.followPath(createSamplePath(sampleOnePose)); // Move to first sample
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        switch (currentState) {
//            case MOVE_TO_FIRST_NEUTRAL_SAMPLE:
//                if (!follower.isBusy()) {
//                    currentState = AutoState.PUSH_FIRST_NEUTRAL_SAMPLE;
//                    startPush();
//                }
//                break;
//
//            case PUSH_FIRST_NEUTRAL_SAMPLE:
//                if (pushFinished()) {
//                    currentState = AutoState.CHECK_FIRST_SAMPLE_FINISHED;
//                }
//                break;
//
//            case CHECK_FIRST_SAMPLE_FINISHED:
//                if (isPushComplete()) {
//                    currentState = AutoState.MOVE_TO_SECOND_NEUTRAL_SAMPLE;
//                    follower.followPath(createSamplePath(sampleTwoPose));
//                }
//                break;
//
//            case MOVE_TO_SECOND_NEUTRAL_SAMPLE:
//                if (!follower.isBusy()) {
//                    currentState = AutoState.PUSH_SECOND_NEUTRAL_SAMPLE;
//                    startPush();
//                }
//                break;
//
//            case PUSH_SECOND_NEUTRAL_SAMPLE:
//                if (pushFinished()) {
//                    currentState = AutoState.CHECK_SECOND_SAMPLE_FINISHED;
//                }
//                break;
//
//            case CHECK_SECOND_SAMPLE_FINISHED:
//                if (isPushComplete()) {
//                    currentState = AutoState.MOVE_TO_THIRD_NEUTRAL_SAMPLE;
//                    follower.followPath(createSamplePath(sampleThreePose));
//                }
//                break;
//
//            case MOVE_TO_THIRD_NEUTRAL_SAMPLE:
//                if (!follower.isBusy()) {
//                    currentState = AutoState.PUSH_THIRD_NEUTRAL_SAMPLE;
//                    startPush();
//                }
//                break;
//
//            case PUSH_THIRD_NEUTRAL_SAMPLE:
//                if (pushFinished()) {
//                    currentState = AutoState.CHECK_THIRD_SAMPLE_FINISHED;
//                }
//                break;
//
//            case CHECK_THIRD_SAMPLE_FINISHED:
//                if (isPushComplete()) {
//                    currentState = AutoState.MOVE_TO_OBSERVATION_ZONE;
//                    follower.followPath(createSamplePath(parkPosition));
//                }
//                break;
//
//            case MOVE_TO_OBSERVATION_ZONE:
//                if (!follower.isBusy()) {
//                    currentState = AutoState.COMPLETE;
//                }
//                break;
//
//            case COMPLETE:
//                follower.breakFollowing();
//                break;
//        }
//
//        telemetry.addData("State", currentState);
//        telemetry.update();
//    }
//
//    private PathChain createSamplePath(Pose samplePose) {
//        return new PathBuilder()
//                .addPath(new BezierLine(
//                        new Point(0, 60, Point.CARTESIAN),
//                        new Point(samplePose.getX(), samplePose.getY(), Point.CARTESIAN)
//                ))
//                .build();
//    }
//
//    @Override
//    public void stop() {
//        telemetry.addData("Auto", "Ended");
//        follower.breakFollowing();
//    }
//
//    private void startPush() {
//        // Add logic for starting a push action
//    }
//
//    private boolean pushFinished() {
//        // Add logic to determine if push is finished
//        return true;
//    }
//
//    private boolean isPushComplete() {
//        // Add logic to check if the push is complete
//        return true;
//    }
//}
