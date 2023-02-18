package com.team2383.diffy.commands.pinkArm.velocity;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.pinkArm.pivot.*;

public class PivotVelocityCommand extends CommandBase {

    private final PivotSubsystem m_pivot;
    private final DoubleSupplier m_speed;

    public PivotVelocityCommand(PivotSubsystem pivot, DoubleSupplier speed) {
        m_pivot = pivot;
        m_speed = speed;

        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        m_pivot.setVelocity(m_speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
