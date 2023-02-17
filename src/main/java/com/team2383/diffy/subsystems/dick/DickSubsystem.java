package com.team2383.diffy.subsystems.dick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DickSubsystem extends SubsystemBase {

    private final CANSparkMax m_dick;

    public DickSubsystem() {

        m_dick = new CANSparkMax(DickConstants.ID, MotorType.kBrushless);
        m_dick.setIdleMode(IdleMode.kCoast);
        m_dick.setInverted(false);

    }

    // Idk what to put in here lol
    @Override
    public void periodic() {
    }

    public void erect(double speed) {
        m_dick.set(speed);
    }
}
