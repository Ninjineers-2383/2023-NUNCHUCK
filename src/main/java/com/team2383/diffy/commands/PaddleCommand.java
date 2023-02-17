package com.team2383.diffy.commands;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.paddle.PaddleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PaddleCommand extends CommandBase {

    private final PaddleSubsystem m_dick;
    private final DoubleSupplier m_erect;

    public PaddleCommand(PaddleSubsystem dick, DoubleSupplier viagra) {

        m_dick = dick;
        m_erect = viagra;

        addRequirements(dick);
    }

    @Override
    public void execute() {
        m_dick.erect(m_erect.getAsDouble());
    }
    
}
