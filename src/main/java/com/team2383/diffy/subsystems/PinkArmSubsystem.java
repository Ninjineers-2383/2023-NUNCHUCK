package com.team2383.diffy.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PinkArmSubsystem extends SubsystemBase {
    private BottomPivotSubsystem m_bottomPivot;
    private TelescopeSubsystem m_telescope;
    private TopPivotSubsystem m_topPivot;

    private DataLog m_log;

    public PinkArmSubsystem(DataLog log) {
        m_log = log;

        m_bottomPivot = new BottomPivotSubsystem(m_log);
        m_telescope = new TelescopeSubsystem(m_log);
        m_topPivot = new TopPivotSubsystem(m_log);

        addChild("Bottom Pivot", m_bottomPivot);
        addChild("Telescope", m_telescope);
        addChild("Top Pivot", m_topPivot);
    }

    @Override
    public void periodic() {
        m_bottomPivot.periodic();
        m_telescope.periodic();
        m_topPivot.periodic();
    }

    @Override
    public void simulationPeriodic() {
        m_bottomPivot.simulate();
        m_telescope.simulate();
        m_topPivot.simulate();
    }

    public void setDesiredState(double desiredBottomAngle, double desiredBottomSpeed, double desiredExtension,
            double desiredExtensionSpeed, double desiredTopAngle, double desiredTopSpeed) {

        m_bottomPivot.setAngle(desiredBottomAngle, desiredBottomSpeed);
        m_telescope.setExtension(desiredExtension, desiredExtensionSpeed);
        m_topPivot.setAngle(desiredTopAngle, desiredTopSpeed);
    }

}
