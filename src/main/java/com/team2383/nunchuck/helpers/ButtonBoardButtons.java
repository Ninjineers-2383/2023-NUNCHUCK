package com.team2383.nunchuck.helpers;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import com.team2383.nunchuck.commands.pinkArm.PinkArmPresetCommand;
import com.team2383.nunchuck.commands.pinkArm.position.PositionConstants;
import com.team2383.nunchuck.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoardButtons {
    private static PositionConstants.PinkPositions[] positions = new PositionConstants.PinkPositions[] {
            PositionConstants.ZERO_POS,
            PositionConstants.TRAVEL_POS,
            PositionConstants.FEED_CHUTE,
            PositionConstants.FEED_CUBE_POS,
            PositionConstants.FEED_CONE_POS,
            PositionConstants.FEED_UPRIGHT,
            PositionConstants.FEED_HIGH,
            PositionConstants.FEED_HIGH_CONE,
            PositionConstants.LOW_SCORE_BACK,
            PositionConstants.MID_SCORE_BACK,
            PositionConstants.HIGH_SCORE_BACK,
            PositionConstants.MID_SCORE_FRONT,
            PositionConstants.HIGH_SCORE_FRONT };

    private static HashMap<String, LoggedDashboardBoolean> buttons = new HashMap<String, LoggedDashboardBoolean>();

    private static ArrayList<String> names = new ArrayList<String>();

    private static void oneHot(String key) {
        for (PositionConstants.PinkPositions position : positions) {
            if (!key.equals(position.name)) {
                if (buttons.get(position.name) == null) {
                    buttons.put(position.name, new LoggedDashboardBoolean(position.name, false));
                }
                buttons.get(position.name).set(false);
            }
        }
    }

    public static Trigger makeNormieButton(String name) {
        var button = new LoggedDashboardBoolean(name, false);
        buttons.put(name, button);
        Trigger smartTrigger = new Trigger(() -> button.get());
        smartTrigger
                .onTrue(new WaitCommand(.01).andThen(new InstantCommand(() -> button.set(false))));
        names.add(name);
        return smartTrigger;
    }

    private static Trigger makePositionButton(PivotSubsystem pivotSubsystem,
            TelescopeSubsystem telescopeSubsystem,
            WristSubsystem wristSubsystem,
            PositionConstants.PinkPositions constant) {
        var button = new LoggedDashboardBoolean(constant.name, false);
        buttons.put(constant.name, button);
        Trigger smartTrigger = new Trigger(() -> button.get());
        smartTrigger.onTrue(new InstantCommand(() -> oneHot(constant.name))
                .andThen(new PinkArmPresetCommand(pivotSubsystem, telescopeSubsystem, wristSubsystem, constant)));
        names.add(constant.name);
        return smartTrigger;
    }

    public static void instantiatePositionalButtons(PivotSubsystem pivotSubsystem,
            TelescopeSubsystem telescopeSubsystem,
            WristSubsystem wristSubsystem) {

        for (PositionConstants.PinkPositions position : positions) {
            makePositionButton(pivotSubsystem, telescopeSubsystem, wristSubsystem, position);
        }
    }
}
