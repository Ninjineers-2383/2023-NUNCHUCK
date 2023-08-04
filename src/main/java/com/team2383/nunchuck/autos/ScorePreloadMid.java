package com.team2383.nunchuck.autos;

import com.team2383.nunchuck.commands.FeederCommand;
import com.team2383.nunchuck.commands.pinkArm.PinkArmPresetCommand;
import com.team2383.nunchuck.commands.pinkArm.position.PositionConstants;
import com.team2383.nunchuck.commands.pinkArm.zero.ZeroTelescope;
import com.team2383.nunchuck.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.feeder.FeederSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScorePreloadMid extends SequentialCommandGroup {
    public ScorePreloadMid(DrivetrainSubsystem m_drivetrain, TelescopeSubsystem telescope, PivotSubsystem pivot,
            WristSubsystem wrist, FeederSubsystem feeder) {
        addCommands(
                new InstantCommand(() -> m_drivetrain.resetHeading()),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.ZERO_POS),
                new ZeroTelescope(telescope),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.MID_SCORE_BACK),
                new FeederCommand(feeder, () -> -1).withTimeout(0.4),
                new FeederCommand(feeder, () -> 0).withTimeout(0.01),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.TRAVEL_POS));
    }
}
