package com.team2383.diffy.commands;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederCommand extends CommandBase {
    private FeederSubsystem m_feederSubsystem;
    private DoubleSupplier m_intake;

    double m_bottomAngleSpeed = 0;
    double m_topAngleSpeed = 0;
    double m_extensionSpeed = 0;

    public FeederCommand(FeederSubsystem feeder,
            DoubleSupplier intake) {
        m_feederSubsystem = feeder;
        m_intake = intake;

        addRequirements(m_feederSubsystem);
    }

    @Override
    public void execute() {
        m_feederSubsystem.setPower(-12 * m_intake.getAsDouble(), -12 * m_intake.getAsDouble());

    }
}
