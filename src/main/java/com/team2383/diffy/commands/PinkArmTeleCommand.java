package com.team2383.diffy.commands;

import java.util.function.IntSupplier;

import com.team2383.diffy.subsystems.PinkArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmTeleCommand extends CommandBase {
    private PinkArmSubsystem m_pinkArm;

    private IntSupplier m_bottomAngle;
    private IntSupplier m_extension;
    private IntSupplier m_topAngle;

    public PinkArmTeleCommand(PinkArmSubsystem pinkArm,
            IntSupplier desiredBottomAngle, IntSupplier desiredExtension, IntSupplier desiredTopAngle) {
        m_pinkArm = pinkArm;
        m_bottomAngle = desiredBottomAngle;
        m_extension = desiredExtension;
        m_topAngle = desiredTopAngle;

        addRequirements(m_pinkArm);
    }

    @Override
    public void execute() {
        double m_bottomAnglePosition = 0;
        double m_topAnglePosition = 0;
        double m_extensionPosition = 0;

        if (m_bottomAngle.getAsInt() == 1) {
            m_bottomAnglePosition += 10;
        } else if (m_bottomAngle.getAsInt() == -1) {
            m_bottomAnglePosition -= 10;
        }

        if (m_extension.getAsInt() == 1) {
            m_extensionPosition += 10;
        } else if (m_extension.getAsInt() == -1) {
            m_extensionPosition -= 10;
        }

        if (m_topAngle.getAsInt() == 1) {
            m_topAnglePosition += 10;
        } else if (m_topAngle.getAsInt() == -1) {
            m_topAnglePosition -= 10;
        }

        m_pinkArm.setDesiredState(m_bottomAnglePosition, 10.0,
                m_extensionPosition, 10.0, m_topAnglePosition, 10.0);
    }
}
