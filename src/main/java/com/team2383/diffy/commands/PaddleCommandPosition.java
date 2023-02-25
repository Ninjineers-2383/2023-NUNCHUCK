package com.team2383.diffy.commands;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.paddle.PaddleSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PaddleCommandPosition extends CommandBase {

    private final PaddleSubsystem m_dick;
    private final Rotation2d m_angle;
    private final DoubleSupplier m_ballsDutyCycle;

    public PaddleCommandPosition(PaddleSubsystem dick, Rotation2d angle, DoubleSupplier ballsDutyCycle) {

        m_dick = dick;
        m_angle = angle;
        m_ballsDutyCycle = ballsDutyCycle;

        addRequirements(dick);
    }

    @Override
    public void execute() {
        m_dick.setPosition(m_angle);
        m_dick.setBallsDutyCycle(m_ballsDutyCycle.getAsDouble());
    }

}
