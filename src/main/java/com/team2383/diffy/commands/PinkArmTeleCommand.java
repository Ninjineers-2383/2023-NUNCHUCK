package com.team2383.diffy.commands;

import java.util.function.BooleanSupplier;

import com.team2383.diffy.subsystems.PinkArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmTeleCommand extends CommandBase {
    private PinkArmSubsystem m_pinkArm;

    private BooleanSupplier m_bottomAngleUp;
    private BooleanSupplier m_bottomAngleDown;
    private BooleanSupplier m_extensionUp;
    private BooleanSupplier m_extensionDown;
    private BooleanSupplier m_topAngleUp;
    private BooleanSupplier m_topAngleDown;

    double m_bottomAngleSpeed = 0;
    double m_topAngleSpeed = 0;
    double m_extensionSpeed = 0;

    public PinkArmTeleCommand(PinkArmSubsystem pinkArm,
            BooleanSupplier bottomAngleUp, BooleanSupplier bottomAngleDown, BooleanSupplier extensionUp, 
            BooleanSupplier extensionDown, BooleanSupplier topAngleUp, BooleanSupplier topAngleDown) {
        m_pinkArm = pinkArm;
        m_bottomAngleDown = bottomAngleDown;
        m_bottomAngleUp = bottomAngleUp;
        m_extensionDown = extensionDown;
        m_extensionUp = extensionUp;
        m_topAngleDown = topAngleDown;
        m_topAngleUp = topAngleUp;
        
        addRequirements(m_pinkArm);
    }

    @Override
    public void execute() {
        if (m_bottomAngleUp.getAsBoolean()) {
            m_bottomAngleSpeed = 1;
        } else if (m_bottomAngleDown.getAsBoolean()) {
            m_bottomAngleSpeed = -1;
        } else {
            m_bottomAngleSpeed = 0;
        }

        if (m_extensionUp.getAsBoolean()) {
            m_extensionSpeed = 0.1;
        } else if (m_extensionDown.getAsBoolean()) {
            m_extensionSpeed = -0.1;
        } else {
            m_extensionSpeed = 0;
        }

        if (m_topAngleUp.getAsBoolean()) {
            m_topAngleSpeed = 1;
        } else if (m_topAngleDown.getAsBoolean()) {
            m_topAngleSpeed = -1;
        }

        m_pinkArm.setDesiredState(m_bottomAngleSpeed,
                m_extensionSpeed, m_topAngleSpeed);
    }
}
