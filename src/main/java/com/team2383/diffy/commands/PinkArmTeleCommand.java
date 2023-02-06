package com.team2383.diffy.commands;

import java.util.function.BooleanSupplier;

import com.team2383.diffy.subsystems.PinkArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmTeleCommand extends CommandBase {
    private PinkArmSubsystem m_pinkArm;

    private BooleanSupplier m_topUp;
    private BooleanSupplier m_topDown;
    private BooleanSupplier m_xUp;
    private BooleanSupplier m_xDown;
    private BooleanSupplier m_yUp;
    private BooleanSupplier m_yDown;

    double m_x = 0.1;
    double m_y = 0.1;
    double m_topAngle = 0.1;

    public PinkArmTeleCommand(PinkArmSubsystem pinkArm,
            BooleanSupplier topUp, BooleanSupplier topDown, BooleanSupplier xUp, 
            BooleanSupplier xDown, BooleanSupplier yUp, BooleanSupplier yDown) {
        m_pinkArm = pinkArm;
        m_topUp = topUp;
        m_topDown = topDown;
        m_xUp = xUp;
        m_xDown = xDown;
        m_yUp = yUp;
        m_yDown = yDown;
        
        addRequirements(m_pinkArm);
    }

    @Override
    public void execute() {
        if (m_xUp.getAsBoolean() && m_x < 1.9) {
            m_x += 0.01;
        } else if (m_xDown.getAsBoolean() && m_x > -1.9) {
            m_x -= 0.01;
        } else if (m_x > 1.9) {
            m_x = 0.1;
        } else if (m_x < -1.9) {
            m_x = -0.1;
        }

        if (m_yUp.getAsBoolean() && m_y < 1.9) {
            m_y += 0.01;
        } else if (m_yDown.getAsBoolean() && m_y > -1.9) {
            m_y -= 0.01;
        } else if (m_y > 1.9) {
            m_y = 0.1;
        } else if (m_y < -1.9) {
            m_y = -0.1;
        }

        if (m_topUp.getAsBoolean() && m_topAngle < 1.9) {
            m_topAngle += 0.01;
        } else if (m_topDown.getAsBoolean() && m_topAngle > -1.9) {
            m_topAngle -= 0.01;
        } else if (m_topAngle > 1.9) {
            m_topAngle = 0.1;
        } else if (m_topAngle < -1.9) {
            m_topAngle = -0.1;
        }

        System.out.println("x: " + m_x + " y: " + m_y + " Top Theta: " + m_topAngle);
        m_pinkArm.setPosition(m_x, m_y, m_topAngle);    
    }
}
