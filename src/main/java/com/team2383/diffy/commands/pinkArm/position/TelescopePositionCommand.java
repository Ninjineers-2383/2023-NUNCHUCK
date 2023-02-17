// TODO: Implement Command

package com.team2383.diffy.commands.pinkArm.position;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.diffy.subsystems.pinkArm.telescope.*;

public class TelescopePositionCommand extends CommandBase {

    private final TelescopeSubsystem m_telescope;
    private final double m_extension;

    public TelescopePositionCommand(TelescopeSubsystem telescope, double extension) {
        m_telescope = telescope;
        m_extension = extension;
        addRequirements(m_telescope);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_telescope.setExtension(m_extension);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_telescope.isAtPosition();
    }
}