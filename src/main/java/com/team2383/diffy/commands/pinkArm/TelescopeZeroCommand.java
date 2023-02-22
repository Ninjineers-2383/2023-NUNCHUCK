package com.team2383.diffy.commands.pinkArm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.diffy.subsystems.pinkArm.telescope.*;

public class TelescopeZeroCommand extends CommandBase {

    private final TelescopeSubsystem m_telescope;

    public TelescopeZeroCommand(TelescopeSubsystem telescope) {
        m_telescope = telescope;
        addRequirements(m_telescope);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_telescope.setVelocity(-1);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_telescope.getZeroState();
    }
}