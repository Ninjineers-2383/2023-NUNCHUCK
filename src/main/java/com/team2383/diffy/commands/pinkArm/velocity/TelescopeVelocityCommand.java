package com.team2383.diffy.commands.pinkArm.velocity;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.pinkArm.telescope.*;

public class TelescopeVelocityCommand extends CommandBase {

    private final TelescopeSubsystem m_telescope;
    private final DoubleSupplier m_speed;

    public TelescopeVelocityCommand(TelescopeSubsystem telescope, DoubleSupplier speed) {
        m_telescope = telescope;
        m_speed = speed;
        addRequirements(m_telescope);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_telescope.setVelocity(m_speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}