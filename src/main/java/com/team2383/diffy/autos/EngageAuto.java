package com.team2383.diffy.autos;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants;
import com.team2383.diffy.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class EngageAuto extends SequentialCommandGroup {
    public EngageAuto(DrivetrainSubsystem m_drivetrainSubsystem, SwerveAutoBuilder autoBuilder,
            PivotSubsystem m_pivotSubsystem) {
        addCommands(
                new FullAutoCommand(m_drivetrainSubsystem, "Engage1", autoBuilder),
                new ParallelDeadlineGroup(
                        new WaitCommand(3),
                        new SequentialCommandGroup(
                                new PivotPositionCommand(m_pivotSubsystem, PositionConstants.LOW_SCORE_BACK.pivot),
                                new PivotPositionCommand(m_pivotSubsystem, PositionConstants.TRAVEL_POS.pivot))),
                new FullAutoCommand(m_drivetrainSubsystem, "Engage2", autoBuilder));
    }
}
