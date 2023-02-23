package com.team2383.diffy.commands.pinkArm.position.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import com.team2383.diffy.commands.pinkArm.position.AutomatedCommands.PositionConstants.*;

import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.WristPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.TelescopePositionCommand;
import edu.wpi.first.math.geometry.Rotation2d;

public class MidGoalCommand extends SequentialCommandGroup {

    public MidGoalCommand(PivotSubsystem pivot, WristSubsystem wrist, TelescopeSubsystem telescope) {
        addCommands(
            new ParallelCommandGroup(
                new PivotPositionCommand(pivot, Rotation2d.fromRadians(PivotPositionConstants.kMidScorePos)),
                new WristPositionCommand(wrist, Rotation2d.fromRadians(WristPositionConstants.kMidScorePos)),
                new TelescopePositionCommand(telescope, TelescopePositionConstants.kRetracted)
            ).withTimeout(4)
        );
    }
    
}
