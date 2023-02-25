package com.team2383.diffy.commands.pinkArm;

import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.TelescopePositionCommand;
import com.team2383.diffy.commands.pinkArm.position.WristPositionCommand;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants.PinkPositions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PinkArmPresetCommand extends SequentialCommandGroup {

    public final PivotSubsystem m_pivot;
    public final TelescopeSubsystem m_telescope;
    public final WristSubsystem m_wrist;

    public final PinkPositions m_PinkPositions;

    // public double m_pivotAngle;
    // public double m_telescopeExtension;
    // public double m_wristAngle;

    public PinkArmPresetCommand(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist,
            PinkPositions pinkPositions) {
        m_pivot = pivot;
        m_telescope = telescope;
        m_wrist = wrist;
        m_PinkPositions = pinkPositions;
        SmartDashboard.putNumber("Stage", 0);

        // m_pivotAngle = pivot.getAngle().getRadians();
        // m_telescopeExtension = telescope.getExtensionInches();
        // m_wristAngle = wrist.getAngle().getRadians();
        addCommands(
                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 1)),
                new ParallelCommandGroup(
                        new TelescopePositionCommand(telescope, 0),
                        new WristPositionCommand(wrist, Rotation2d.fromDegrees(0))),
                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 2)),
                new PivotPositionCommand(pivot, m_PinkPositions.pivot),
                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 3)),
                new ParallelCommandGroup(
                        new TelescopePositionCommand(telescope, m_PinkPositions.extension),
                        new WristPositionCommand(wrist, m_PinkPositions.wrist)),
                new InstantCommand(() -> SmartDashboard.putNumber("Stage", 4)));

    }

}
