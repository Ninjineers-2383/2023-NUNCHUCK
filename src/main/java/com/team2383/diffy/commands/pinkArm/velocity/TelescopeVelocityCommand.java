package com.team2383.diffy.commands.pinkArm.velocity;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.pinkArm.telescope.*;

public class TelescopeVelocityCommand extends CommandBase {

    private final TelescopeSubsystem m_telescope;
    private final DoubleSupplier m_extension;

    public TelescopeVelocityCommand(TelescopeSubsystem telescope, DoubleSupplier extension) {
        m_telescope = telescope;
        m_extension = extension;
        addRequirements(m_telescope);
    }

    @Override
    public void execute() {
        m_telescope.setVelocity(m_extension.getAsDouble());
    }
}