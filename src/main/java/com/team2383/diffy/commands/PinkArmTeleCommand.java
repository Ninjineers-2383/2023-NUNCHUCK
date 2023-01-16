package com.team2383.diffy.commands;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.PinkArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmTeleCommand extends CommandBase {
    private PinkArmSubsystem m_pinkArm;

    private DoubleSupplier m_bottomAngle;
    private DoubleSupplier m_extension;
    private DoubleSupplier m_topAngle;

    public PinkArmTeleCommand(PinkArmSubsystem pinkArm, DoubleSupplier desiredBottomAngle, DoubleSupplier desiredExtension, DoubleSupplier desiredTopAngle) {
        m_pinkArm = pinkArm;
        m_bottomAngle = desiredBottomAngle;
        m_extension = desiredExtension;
        m_topAngle = desiredTopAngle;

        addRequirements(m_pinkArm);
    }

    @Override
    public void execute() {
        m_pinkArm.setDesiredState(m_bottomAngle.getAsDouble(), 10.0, 
            m_extension.getAsDouble(), 10.0, m_topAngle.getAsDouble(), 10.0);
    }
}
