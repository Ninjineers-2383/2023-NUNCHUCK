package com.team2383.diffy.helpers;

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
            PositionConstants.FEED_CUBE_POS,
            PositionConstants.FEED_CUBE_POS,
            PositionConstants.LOW_SCORE_POS,
            PositionConstants.MID_SCORE_POS,
            PositionConstants.HIGH_SCORE_POS };

    private static void publishDashboard(String key) {
        for (PositionConstants.PinkPositions position : positions) {
            if (!key.equals(position.name)) {
                SmartDashboard.putBoolean(position.name, false);
            }
        }
    }

    public static Trigger makeNormieButton(String name) {
        Trigger smartTrigger = new Trigger(() -> SmartDashboard.getBoolean(name, false));
        smartTrigger
                .onTrue(new WaitCommand(.5).andThen(new InstantCommand(() -> SmartDashboard.putBoolean(name, false))));
        return smartTrigger;
    }

    private static Trigger makePositionButton(PivotSubsystem pivotSubsystem,
            TelescopeSubsystem telescopeSubsystem,
            WristSubsystem wristSubsystem,
            PositionConstants.PinkPositions constant) {
        Trigger smartTrigger = new Trigger(() -> SmartDashboard.getBoolean(constant.name, false));
        smartTrigger.onTrue(new InstantCommand(() -> publishDashboard(constant.name))
                .andThen(new PinkArmPresetCommand(pivotSubsystem, telescopeSubsystem, wristSubsystem, constant)));
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
