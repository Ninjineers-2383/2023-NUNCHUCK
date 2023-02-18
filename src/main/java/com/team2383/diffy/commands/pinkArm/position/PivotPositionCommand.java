// TODO: Implement Command
package com.team2383.diffy.commands.pinkArm.position;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.diffy.subsystems.pinkArm.pivot.*;

public class PivotPositionCommand extends CommandBase {

    private final PivotSubsystem m_pivot;
    private final double m_angle;

    public PivotPositionCommand(PivotSubsystem pivot, double angle) {
        m_pivot = pivot;
        m_angle = angle;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        // Keeping extension at 0, seems sketchy to rotate and extend at the same time
        m_pivot.setAngle(m_angle);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_pivot.isAtPosition();
    }
}

