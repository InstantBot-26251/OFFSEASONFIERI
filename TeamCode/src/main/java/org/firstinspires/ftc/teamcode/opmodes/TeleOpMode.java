package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.RobotStatus.Alliance.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Fieri;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.util.commands.AllianceToggler;

@Config
@TeleOp(name = "Period of Manual Control")
public class TeleOpMode extends LinearOpMode {
    private final Fieri fieri = Fieri.getInstance();
    private AllianceToggler allianceToggler;

    @Override
    public void runOpMode() throws InterruptedException {

        // INIT_LOOP
        while (opModeInInit()) {
            // INIT
            fieri.teleopInit(telemetry, hardwareMap, gamepad1, gamepad2);
            allianceToggler.update();
            allianceToggler.giveAllianceFeedback();


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