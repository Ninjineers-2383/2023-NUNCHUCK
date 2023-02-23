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

public class GroundFeedCommand extends SequentialCommandGroup {

    public GroundFeedCommand(PivotSubsystem pivot, WristSubsystem wrist, TelescopeSubsystem telescope) {
        addCommands(
            new ParallelCommandGroup(
                new PivotPositionCommand(pivot, Rotation2d.fromRadians(PivotPositionConstants.kFeedGroundPos)),
                new WristPositionCommand(wrist, Rotation2d.fromRadians(WristPositionConstants.kFeedGroundPos)),
                new TelescopePositionCommand(telescope, TelescopePositionConstants.kRetracted)
            ).withTimeout(4)
        );
    }
    
}
