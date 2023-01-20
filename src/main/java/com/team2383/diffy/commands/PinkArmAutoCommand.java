package com.team2383.diffy.commands;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.PinkArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmAutoCommand extends CommandBase {
    private PinkArmSubsystem m_pinkArm;

    private DoubleSupplier m_bottomAngle;
    private DoubleSupplier m_bottomSpeed;
    private DoubleSupplier m_extension;
    private DoubleSupplier m_extensionSpeed;
    private DoubleSupplier m_topAngle;
    private DoubleSupplier m_topSpeed;

    public PinkArmAutoCommand(PinkArmSubsystem pinkArm, DoubleSupplier desiredBottomAngle, DoubleSupplier desiredBottomSpeed, DoubleSupplier desiredExtension, 
        DoubleSupplier desiredExtensionSpeed, DoubleSupplier desiredTopAngle, DoubleSupplier desiredTopSpeed) {
        m_pinkArm = pinkArm;
        m_bottomAngle = desiredBottomAngle;
        m_bottomSpeed = desiredBottomSpeed;
        m_extension = desiredExtension;
        m_extensionSpeed = desiredExtensionSpeed;
        m_topAngle = desiredTopAngle;
        m_topSpeed = desiredTopSpeed;

        addRequirements(m_pinkArm);
    }

    @Override
    public void execute() {
        // m_pinkArm.setDesiredState(m_bottomAngle.getAsDouble(), 
        //     m_extension.getAsDouble(), m_extensionSpeed.getAsDouble(), m_topAngle.getAsDouble(), m_topSpeed.getAsDouble());
    }
}
