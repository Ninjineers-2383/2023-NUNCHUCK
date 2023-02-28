package com.team2383.diffy.commands.pinkArm;

import com.team2383.diffy.commands.FeederCommand;
import com.team2383.diffy.commands.PaddleCommandPosition;
import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants;
import com.team2383.diffy.commands.pinkArm.position.WristPositionCommand;
import com.team2383.diffy.subsystems.paddle.PaddleSubsystem;
import com.team2383.diffy.subsystems.pinkArm.feeder.FeederSubsystem;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TravelCommand extends SequentialCommandGroup {
    public TravelCommand(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist,
            PaddleSubsystem paddle, FeederSubsystem feeder) {
        addCommands(
                // Stage 1: Moves pink arm to travel position
                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 1)),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.FEED_PADDLE_POS_INIT)
                        .withTimeout(2),
                // Stage 2: Moves paddle down to position
                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 2)),
                new FeederCommand(feeder, () -> 0.5)
                        .withTimeout(0.1),
                // Stage 3: Turns feeder on
                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 3)),
                new PaddleCommandPosition(paddle, Rotation2d.fromDegrees(50))
                        .withTimeout(1.5),
                // Stage 4: Moves pivot in to feed cone
                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 4)),
                new ParallelDeadlineGroup(
                    new PivotPositionCommand(pivot, Rotation2d.fromDegrees(-12))
                        .withTimeout(2.5),
                    new WristPositionCommand(wrist, Rotation2d.fromDegrees(-52))),
                    
                // Stage 5: Turns feeder off
                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 5)),
                new FeederCommand(feeder, () -> 0)
                        .withTimeout(0.1),

                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 6)),
                new PivotPositionCommand(pivot, Rotation2d.fromDegrees(-41))
                        .withTimeout(1.5),

                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 7)),
                new PaddleCommandPosition(paddle, Rotation2d.fromDegrees(70))
                        .withTimeout(1),

                new InstantCommand(() -> SmartDashboard.putNumber("Travel Command Stage", 8)),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.TRAVEL_POS)
                        .withTimeout(2));

    }
}
