package com.team2383.diffy.commands.pinkArm.position;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.pinkArm.pivot.*;

public class PivotPositionCommand extends CommandBase {

    private final PivotSubsystem m_pivot;
    private final Rotation2d m_angle;
    private final DoubleSupplier m_extension;

    private boolean success = false;

    public PivotPositionCommand(PivotSubsystem pivot, Rotation2d angle, DoubleSupplier extension) {
        m_pivot = pivot;
        m_angle = angle;
        m_extension = extension;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        success = false;
    }

    public void execute() {
        // Keeps attempting to move arm until success
        if (!success) {
            success = m_pivot.setAngle(m_angle, m_extension.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_pivot.isAtPosition();
    }
}

