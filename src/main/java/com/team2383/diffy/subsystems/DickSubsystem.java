package com.team2383.diffy.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.team2383.diffy.Constants.DickConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DickSubsystem extends SubsystemBase{

    private final CANSparkMax m_dick;

    public DickSubsystem() {

        m_dick = new CANSparkMax(DickConstants.port, MotorType.kBrushless);
        m_dick.setIdleMode(IdleMode.kCoast);
        m_dick.setInverted(false);

    }

    // Idk what to put in here lol
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Dick Moving: ", m_dick.get());
    }
    
    public void erect(double speed) {
        m_dick.set(speed);
    }
}
