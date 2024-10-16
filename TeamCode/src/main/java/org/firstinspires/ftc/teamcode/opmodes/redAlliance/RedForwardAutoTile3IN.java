package org.firstinspires.ftc.teamcode.opmodes.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmodes.util.AutoState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.ArmAndIntakeFunctions;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.ScoreHighBasket;

@Autonomous(name = "Blue Alliance Auto Position 1")
public class RedForwardAutoTile3IN extends OpMode {

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
                new Point(19, 125.5, Point.CARTESIAN),     // Start point
                new Point(130.5, 95, Point.CARTESIAN),   // Control point
                new Point(134.22, 125.22, Point.CARTESIAN)     // End point
        );

        // Create a path to the scoring zone using the BezierCurve
        scoringPath = new PathBuilder()
                .addPath(scoringCurve) // Add BezierCurve as a path
                .build();

        scoringCurve = new BezierCurve(
                new Point(138, 85, Point.CARTESIAN),      // Start point
                new Point(130, 65, Point.CARTESIAN),     // Control point
                new Point(128.58, 16.13, Point.CARTESIAN)       // End point
        );

        //Initialize score
        score = new ScoreHighBasket(arm, intake, gamepad2, functions);

        // Initialize hardware components
        arm = hardwareMap.get(Arm.class, "armMotor");
        intake = hardwareMap.get(Intake.class, "intakeServo");

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
        follower.setStartingPose(new Pose(138, 60, 0)); // Starting position (x = 138, y = 60, heading = 0°)


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

        switch (currentState) {
            case MOVE_TO_SCORING_ZONE:
                // Check if the robot has reached the scoring zone
                if (!follower.isBusy()) {
                    currentState = AutoState.SCORE_HIGH_BASKET;
                }
                break;

            case SCORE_HIGH_BASKET:
                telemetry.addData("Arm Position", arm.getRotatedArmPosition());
                telemetry.addData("Lift Position", arm.getEncoderValue());
                score.execute();
                currentState = AutoState.CHECK_SCORING_FINISHED;

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

    @Override
    public void stop() {
        // When the OpMode ends, ensure all actions are stopped
        follower.breakFollowing(); // Use breakFollowing to stop all motors
    }
}
