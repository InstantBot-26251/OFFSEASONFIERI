package org.firstinspires.ftc.teamcode.opmodes.auto.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.util.AutoStateTO;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Red Observation Zone Auto")
public class RedToObservationAuto extends OpMode {
//for all our next seasons we should have nicholas as part of all our repos...:)))))
    private Follower follower;
    private BezierCurve observationCurve;
    private PathChain observationPath;

    private AutoStateTO currentState = AutoStateTO.MOVE_TO_OBSERVATION_ZONE;
    private long startTime;

    @Override
    public void init() {
        // Create a BezierCurve for the path to the observation zone
        observationCurve = new BezierCurve(
                new Point(138, 85, Point.CARTESIAN),     // Start point
                new Point(137, 110, Point.CARTESIAN),   // Control point
                new Point(134.22, 125.22, Point.CARTESIAN)     // End point
        );

        // Create a path to the observation zone using the BezierCurve
        observationPath = new PathBuilder()
                .addPath(observationCurve) // Add BezierCurve as a path
                .build();

        // Initialize Follower with hardware components
        follower = new Follower(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("State", currentState);
        telemetry.addData("Time", System.currentTimeMillis() - startTime);
    }

    @Override
    public void start() {
        currentState = AutoStateTO.MOVE_TO_OBSERVATION_ZONE;
        startTime = System.currentTimeMillis(); // Record the start time

        // Set the initial position of the robot
        follower.setStartingPose(new Pose(138, 85, 0)); // Starting position (x = 138, y = 85, heading = 0°)


        // Follow the path to the observation zone
        follower.followPath(observationPath);
    }

    @Override
    public void loop() {
        // Continuously update the follower to execute the path
        follower.update();

        // Check if 30 seconds have passed
        if (System.currentTimeMillis() - startTime >= 30000) {
            currentState = AutoStateTO.COMPLETE; // Transition to COMPLETE state
        }

        switch (currentState) {
            case MOVE_TO_OBSERVATION_ZONE:
                // Check if the robot has reached the scoring zone
                if (!follower.isBusy()) {
                    currentState = AutoStateTO.COMPLETE;
                    break;

                }
            case COMPLETE:
                telemetry.addData("Path", "Complete");
                follower.breakFollowing(); // Stop following the path
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.update();
    }

    @Override
    public void stop () {
        // When the OpMode ends, ensure all actions are stopped
        follower.breakFollowing(); // Use breakFollowing to stop all motors
    }
}