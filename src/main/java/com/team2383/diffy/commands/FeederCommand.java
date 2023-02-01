package com.team2383.diffy.commands;

import java.util.function.BooleanSupplier;

import com.team2383.diffy.subsystems.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederCommand extends CommandBase {
    private FeederSubsystem m_feederSubsystem;
    private BooleanSupplier m_intake;
    private BooleanSupplier m_outtake;

    double m_bottomAngleSpeed = 0;
    double m_topAngleSpeed = 0;
    double m_extensionSpeed = 0;

    public FeederCommand(FeederSubsystem feeder,
            BooleanSupplier intake, BooleanSupplier outtake) {
        m_feederSubsystem = feeder;
        m_intake = intake;
        m_outtake = outtake;
        
        addRequirements(m_feederSubsystem);
    }

    @Override
    public void execute() {
        if (m_intake.getAsBoolean()) {
            m_feederSubsystem.setPower(-1000, 1000);
        } else if (m_outtake.getAsBoolean()) {
            m_feederSubsystem.setPower(1000, -1000);
        } else {
            m_feederSubsystem.setPower(0, 0);
        }

    }
}

