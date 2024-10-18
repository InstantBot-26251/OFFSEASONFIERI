package org.firstinspires.ftc.teamcode.opmodes.miscellaneous;


import org.firstinspires.ftc.teamcode.opmodes.util.AutoStateM;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Empty Auto")
public class EmptyAuto extends OpMode {
    private long startTime;

private AutoStateM currentState = AutoStateM.WAIT_FOR_30_SECONDS;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
        telemetry.addData("Auto Mode", "Empty");
        telemetry.addData("Time", System.currentTimeMillis() - startTime);
    }

    @Override
    public void start() {
        currentState = AutoStateM.WAIT_FOR_30_SECONDS;
        startTime = System.currentTimeMillis(); // Record the start time

    }

    @Override
    public void loop() {
        // Check if 30 seconds have passed
        if (System.currentTimeMillis() - startTime >= 30000) {
            currentState = AutoStateM.COMPLETE; // Transition to COMPLETE state
        }
            switch (currentState) {
                case WAIT_FOR_30_SECONDS:
                    break;

                case COMPLETE:
                    telemetry.addData("Path", "Complete");
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.update();
        }
    }
