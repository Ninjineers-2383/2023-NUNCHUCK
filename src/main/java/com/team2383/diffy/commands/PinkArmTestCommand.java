package com.team2383.diffy.commands;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.PinkArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmTestCommand extends CommandBase {
    private PinkArmSubsystem m_pinkArm;

    private final DoubleSupplier m_pivot;
    private final DoubleSupplier m_extension;
    private final DoubleSupplier m_wrist;

    double m_x = 0.1;
    double m_y = 0.1;
    double m_topAngle = 0.1;

    public PinkArmTestCommand(PinkArmSubsystem pinkArm,
            DoubleSupplier pivot, DoubleSupplier extension, DoubleSupplier wrist) {
        m_pinkArm = pinkArm;
        m_pivot = pivot;
        m_extension = extension;
        m_wrist = wrist;
        addRequirements(m_pinkArm);
    }

    @Override
    public void execute() {
        

        m_pinkArm.setDesiredVelocities(m_pivot.getAsDouble(), m_extension.getAsDouble(), m_wrist.getAsDouble());
    }
}
