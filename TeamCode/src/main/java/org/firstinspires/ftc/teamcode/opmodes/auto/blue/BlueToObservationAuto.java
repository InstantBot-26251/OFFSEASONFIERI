package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.util.AutoStateTO;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Blue Observation Zone Auto")
public class BlueToObservationAuto extends OpMode {

    private Follower follower;
    private BezierCurve observationCurve;
    private PathChain observationPath;

    private AutoStateTO currentState = AutoStateTO.MOVE_TO_OBSERVATION_ZONE;
    private long startTime;

    @Override
    public void init() {
        // Create a BezierCurve for the path to the observation zone
        observationCurve = new BezierCurve(
                new Point(0, 60, Point.CARTESIAN),     // Start point
                new Point(4, 50, Point.CARTESIAN),   // Control point
                new Point(7.6, 16.13, Point.CARTESIAN)     // End point
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
        follower.setStartingPose(new Pose(0, 60, 0)); // Starting position (x = 0, y = 60, heading = 0°)


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