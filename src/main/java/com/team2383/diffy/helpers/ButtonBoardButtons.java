package com.team2383.diffy.helpers;

import java.util.ArrayList;

import com.team2383.diffy.commands.pinkArm.PinkArmPresetCommand;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static ArrayList<String> names = new ArrayList<String>();

    private static void publishDashboard(String key) {
        for (PositionConstants.PinkPositions position : positions) {
            if (!key.equals(position.name)) {
                SmartDashboard.putBoolean(position.name, false);
            }
        }
    }

    public static void setAllFalse() {
        for (String name : names) {
            SmartDashboard.putBoolean(name, false);
        }
    }

    public static Trigger makeNormieButton(String name) {
        SmartDashboard.setDefaultBoolean(name, false);
        Trigger smartTrigger = new Trigger(() -> SmartDashboard.getBoolean(name, false));
        smartTrigger
                .onTrue(new WaitCommand(.01).andThen(new InstantCommand(() -> SmartDashboard.putBoolean(name, false))));
        names.add(name);
        return smartTrigger;
    }

    private static Trigger makePositionButton(PivotSubsystem pivotSubsystem,
            TelescopeSubsystem telescopeSubsystem,
            WristSubsystem wristSubsystem,
            PositionConstants.PinkPositions constant) {
        SmartDashboard.setDefaultBoolean(constant.name, false);
        Trigger smartTrigger = new Trigger(() -> SmartDashboard.getBoolean(constant.name, false));
        smartTrigger.onTrue(new InstantCommand(() -> publishDashboard(constant.name))
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
