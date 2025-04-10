package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.RobotStatus.Alliance.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Fieri;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;

@Config
@TeleOp(name = "Period of Manual Control")
public class TeleOpMode extends LinearOpMode {
    private final Fieri fieri = Fieri.getInstance();
    private boolean lastToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // INIT_LOOP
        while (opModeInInit()) {
            // INIT
            fieri.teleopInit(telemetry, hardwareMap, gamepad1, gamepad2);

            boolean togglePressed = gamepad1.start;

            if (togglePressed && !lastToggle) {
                switch (RobotStatus.alliance) {
                    case NONE:
                    case RED:
                        RobotStatus.alliance = BLUE;
                        break;
                    case BLUE:
                        RobotStatus.alliance = RED;
                        break;
                }
            }
            lastToggle = togglePressed;

            telemetry.addData("Alliance", RobotStatus.alliance);
            telemetry.addData("Status", RobotStatus.robotState);
            telemetry.update();

        }


        // LOOP
        while (opModeIsActive()) {
            fieri.periodic();
        }

        // END
        fieri.disabledInit();
    }
}