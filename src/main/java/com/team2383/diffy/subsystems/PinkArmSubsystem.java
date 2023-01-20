package com.team2383.diffy.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PinkArmSubsystem extends SubsystemBase {
    private BottomPivotModule m_bottomPivot;
    private TelescopeModule m_telescope;
    private TopPivotModule m_topPivot;

    private DataLog m_log;

    public PinkArmSubsystem(DataLog log) {
        m_log = log;

        m_bottomPivot = new BottomPivotModule(m_log);
        m_telescope = new TelescopeModule(m_log);
        m_topPivot = new TopPivotModule(m_log);

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

    public void setDesiredState(double desiredBottomSpeed,
            double desiredExtensionSpeed, double desiredTopSpeed) {
        m_bottomPivot.setAngle(desiredBottomSpeed, m_telescope.getExtension());
        m_telescope.setExtension(desiredExtensionSpeed);
        m_topPivot.setAngle(desiredTopSpeed);
    }

}
