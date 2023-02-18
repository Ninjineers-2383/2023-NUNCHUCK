package com.team2383.diffy.commands.pinkArm.velocity;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.pinkArm.wrist.*;

public class WristVelocityCommand extends CommandBase {

    private final WristSubsystem m_wrist;
    private final DoubleSupplier m_speed;

    public WristVelocityCommand(WristSubsystem wrist, DoubleSupplier speed) {
        m_wrist = wrist;
        m_speed = speed;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        m_wrist.setVelocity(m_speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}