package com.team2383.diffy.commands.pinkArm;

import java.util.function.BooleanSupplier;

import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.TelescopePositionCommand;
import com.team2383.diffy.commands.pinkArm.position.WristPositionCommand;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants.PinkPositions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class PinkArmPresetCommand extends SequentialCommandGroup {

    public final PivotSubsystem m_pivot;
    public final TelescopeSubsystem m_telescope;
    public final WristSubsystem m_wrist;

    public final PinkPositions m_PinkPositions;

    public PinkArmPresetCommand(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist,
            PinkPositions pinkPositions) {
        m_pivot = pivot;
        m_telescope = telescope;
        m_wrist = wrist;
        m_PinkPositions = pinkPositions;
        SmartDashboard.putNumber("Stage", 0);
        BooleanSupplier m_needToReset = () -> (((Math.signum(m_pivot.getAngle().getDegrees()) != Math
                .signum(pinkPositions.pivot.getDegrees()))
                || (m_PinkPositions.pivot.getDegrees() > -60 && m_PinkPositions.pivot.getDegrees() < 24))
                && (m_pivot.getAngle().getDegrees() < -60 || m_pivot.getAngle().getDegrees() > 24));
        addCommands(
                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 1)),
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> m_telescope.getExtensionInches() < 12),
                                                new ConditionalCommand(
                                                        new PivotPositionCommand(pivot, Rotation2d.fromDegrees(24)),
                                                        new PivotPositionCommand(pivot, Rotation2d.fromDegrees(-60)),
                                                        () -> Math.signum(m_pivot.getAngle().getDegrees()) == 1)),
                                        new TelescopePositionCommand(telescope, 0),
                                        new WristPositionCommand(wrist, Rotation2d.fromDegrees(0))),
                                new WaitCommand(0),
                                m_needToReset),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 2)),
                                new PivotPositionCommand(pivot, m_PinkPositions.pivot),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> Math
                                                .signum(pivot.getAngle().getRadians()) == Math
                                                        .signum(m_PinkPositions.pivot.getRadians())
                                                || m_PinkPositions.pivot.getDegrees() == 0),
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 3)),
                                                new TelescopePositionCommand(telescope, m_PinkPositions.extension),
                                                new WristPositionCommand(wrist, m_PinkPositions.wrist))))),
                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 4)));

    }

}
