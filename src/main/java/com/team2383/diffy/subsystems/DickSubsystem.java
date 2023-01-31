package com.team2383.diffy.subsystems;

import 
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DickSubsystem extends SubsystemBase{

    private final VictorSPX m_dick;

    public DickSubsystem() {

        m_dick = new VictorSPX(DickConstants.port);
        m_dick.setNeutralMode(NeutralMode.Coast);
        m_dick.setInverted(false);

        CurrentLimitsConfigs supply = new CurrentLimitsConfigs();
        supply.SupplyCurrentLimit = 20;
        supply.SupplyCurrentLimitEnable = true;

        m_dick.get
    
    }

    @Override
    public void periodic() {
    }
    
    public void spin(double speed) {
        if(dickLog.)
            m_dick.set(ControlMode.PercentOutput, speed);
    }
}
