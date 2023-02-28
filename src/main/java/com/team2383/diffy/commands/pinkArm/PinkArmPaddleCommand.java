package com.team2383.diffy.commands.pinkArm;

import com.team2383.diffy.commands.FeederCommand;
import com.team2383.diffy.commands.PaddleCommandPosition;
import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants;
import com.team2383.diffy.subsystems.paddle.PaddleSubsystem;
import com.team2383.diffy.subsystems.pinkArm.feeder.FeederSubsystem;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PinkArmPaddleCommand extends SequentialCommandGroup {
    public PinkArmPaddleCommand(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist,
            PaddleSubsystem paddle, FeederSubsystem feeder) {
        addCommands(new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.FEED_PADDLE_POS_INIT),
                new PaddleCommandPosition(paddle, Rotation2d.fromDegrees(10)),
                new FeederCommand(feeder, () -> 0.5).withTimeout(0.5),
                new PivotPositionCommand(pivot, Rotation2d.fromDegrees(0)),
                new FeederCommand(feeder, () -> 0).withTimeout(0.5),
                new PivotPositionCommand(pivot, Rotation2d.fromDegrees(10)),
                new PaddleCommandPosition(paddle, Rotation2d.fromDegrees(-5)),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.TRAVEL_POS));
    }

}
