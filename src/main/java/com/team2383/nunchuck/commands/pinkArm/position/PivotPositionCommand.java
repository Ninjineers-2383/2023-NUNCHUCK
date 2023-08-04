package com.team2383.nunchuck.commands.pinkArm.position;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.nunchuck.subsystems.pinkArm.pivot.*;

public class PivotPositionCommand extends CommandBase {

    private final PivotSubsystem m_pivot;
    private final Rotation2d m_angle;

    private boolean success = false;

    public PivotPositionCommand(PivotSubsystem pivot, Rotation2d angle) {
        m_pivot = pivot;
        m_angle = angle;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        success = false;
    }

    public void execute() {
        // Keeps attempting to move arm until success
        if (!success) {
            success = m_pivot.setGoal(m_angle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        DataLogManager.log("Pivot Position Command Finished");
    }

    @Override
    public boolean isFinished() {
        return m_pivot.isAtPosition();
    }
}
