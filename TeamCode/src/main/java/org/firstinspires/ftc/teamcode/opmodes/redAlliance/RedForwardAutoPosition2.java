package org.firstinspires.ftc.teamcode.opmodes.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.ArmAndIntakeFunctions;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.opmodes.util.AutoState;
import org.firstinspires.ftc.teamcode.util.ScoreHighBasket;


@Autonomous(name = "Red Forward Autonomous", group = "Autonomous")
public class RedForwardAutoPosition2 extends OpMode {

    private ScoreHighBasket score;
    private Follower follower;
    private ArmAndIntakeFunctions functions;
    private Arm arm;
    private Intake intake;
    private BezierCurve scoringCurve;
    private PathChain scoringPath;
    private BezierCurve observationCurve;
    private PathChain observationPath;

    private AutoState currentState = AutoState.MOVE_TO_SCORING_ZONE;
    private long startTime;

    @Override
    public void init() {
        // Create a path to the observation zone using the BezierCurve
        observationPath = new PathBuilder()
                .addPath(observationCurve) // Add BezierCurve as a path
                .build();

        // Create a BezierCurve for the path to the observation zone
        observationCurve = new BezierCurve(
                new Point(60, 126, Point.CARTESIAN),     // Start from scoring position
                new Point(48, 114, Point.CARTESIAN),   // Control point (adjust as needed)
                new Point(24, 96, Point.CARTESIAN)     // End point (24 inches to the side)
        );

        // Create a path to the scoring zone using the BezierCurve
        scoringPath = new PathBuilder()
                .addPath(scoringCurve) // Add BezierCurve as a path
                .build();

        scoringCurve = new BezierCurve(
                new Point(138, 60, Point.CARTESIAN),      // Starting point
                new Point(128, 50, Point.CARTESIAN),     // Control point (adjust as needed)
                new Point(127.4, 15.18, Point.CARTESIAN)       // End point (24 inches forward)
        );

        //Initialize score
        score = new ScoreHighBasket(arm, intake, gamepad2, functions);

        // Initialize hardware components
        arm = hardwareMap.get(Arm.class, "arm");
        intake = hardwareMap.get(Intake.class, "intake");

        // Initialize Follower and ArmAndIntakeFunctions with hardware components
        follower = new Follower(hardwareMap);
        functions = new ArmAndIntakeFunctions(arm, intake, gamepad2);

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {
        telemetry.addData("State", currentState);
    }

    @Override
    public void start() {
        currentState = AutoState.MOVE_TO_SCORING_ZONE;
        startTime = System.currentTimeMillis(); // Record the start time

        // Set the initial position of the robot
        follower.setStartingPose(new Pose(18, 126, 0)); // Starting position (x = 18, y = 126, heading = 0°)


        // Follow the path to the scoring zone
        follower.followPath(scoringPath);
    }

    @Override
    public void loop() {
        // Continuously update the follower to execute the path
        follower.update();

        // Check if 30 seconds have passed
        if (System.currentTimeMillis() - startTime >= 30000) {
            currentState = AutoState.COMPLETE; // Transition to COMPLETE state
        }

        // Add telemetry for current state
        telemetry.addData("Current State", currentState);
        telemetry.addData("Time Elapsed", (System.currentTimeMillis() - startTime) / 1000 + " seconds");

        switch (currentState) {
            case MOVE_TO_SCORING_ZONE:
                // Check if the robot has reached the scoring zone
                telemetry.addData("Follower Busy", follower.isBusy());
                if (!follower.isBusy()) {
                    currentState = AutoState.SCORE_HIGH_BASKET;
                }
                break;

            case SCORE_HIGH_BASKET:
                // Use ArmAndIntakeFunctions to score the preloaded game piece in the high basket
                telemetry.addData("Arm Position", arm.getRotatedArmPosition());
                score.execute();
                currentState = AutoState.CHECK_SCORING_FINISHED;
                break;

            case CHECK_SCORING_FINISHED:
                // Check if the high basket scoring is finished
                if (functions.isFinished()) {
                    currentState = AutoState.MOVE_TO_OBSERVATION_ZONE; // Move to the next state
                }
                break;

            case MOVE_TO_OBSERVATION_ZONE:
                // Follow the path to the observation zone
                follower.followPath(observationPath);
                currentState = AutoState.COMPLETE; // Move to complete after following the path
                break;


            case COMPLETE:
                telemetry.addData("Path", "Complete");
                follower.breakFollowing(); // Stop following the path
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.update();
    }

    public void stop() {
        // When the OpMode ends, ensure all actions are stopped
        follower.breakFollowing(); // Use breakFollowing to stop all motors
    }
}
