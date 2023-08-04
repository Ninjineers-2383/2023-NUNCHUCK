package com.team2383.nunchuck.commands.pinkArm.velocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.team2383.nunchuck.subsystems.pinkArm.pivot.*;

public class PivotVelocityCommand extends CommandBase {

    private final PivotSubsystem m_pivot;
    private final Supplier<Rotation2d> m_angle;

    public PivotVelocityCommand(PivotSubsystem pivot, Supplier<Rotation2d> angle) {
        m_pivot = pivot;
        m_angle = angle;
        addRequirements(m_pivot);
    }

    public void execute() {
        m_pivot.setVelocity(m_angle.get());
    }
}
