package org.firstinspires.ftc.teamcode.opmodes.util;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.ArmAndIntakeFunctions;
import org.firstinspires.ftc.teamcode.util.CollectSample;
import org.firstinspires.ftc.teamcode.util.LevelTwoAscent;
import org.firstinspires.ftc.teamcode.util.ScoreHighBasket;

public class Gamepad2InputHandler {
    private Gamepad gamepad2;
    private CollectSample collection;
    private ScoreHighBasket scorehighbasket;
    private LevelTwoAscent ascent;
    private ArmAndIntakeFunctions functions;

    public Gamepad2InputHandler(Gamepad gamepad2, CollectSample collection, ScoreHighBasket scorehighbasket, LevelTwoAscent ascent, ArmAndIntakeFunctions functions) {
        this.gamepad2 = gamepad2;
        this.collection = collection;
        this.scorehighbasket = scorehighbasket;
        this.ascent = ascent;
        this.functions = functions;
    }

    public void handleInputs() {
        // Handle collection with 'a' button
        if (gamepad2.a) {
            collection.drive();
        }

        // Handle scoring in the high basket with the 'b' button
        if (gamepad2.b && !scorehighbasket.isScoringInProgress()) {
            scorehighbasket.execute();
        }

        // Continuous scoring check
        if (scorehighbasket.isScoringInProgress()) {
            scorehighbasket.execute();
            if (scorehighbasket.isFinished()) {
                // Handle post-scoring logic here
            }
        }

        // Handle level two ascent with 'y' button
        if (gamepad2.y) {
            ascent.drive();
        }

        // Arm positioning with D-pad
        if (gamepad2.dpad_down) {
            functions.armToDownPosition();
        }

        if (gamepad2.dpad_up) {
            functions.armTo90Degrees();
        }

    }

}