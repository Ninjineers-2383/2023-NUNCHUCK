package com.team2383.diffy.subsystems;

import com.team2383.diffy.helpers.PinkArmKinematics;
import com.team2383.diffy.helpers.PinkArmState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PinkArmSubsystem extends SubsystemBase {
    private final BottomPivotModule m_bottomPivot;
    private final TelescopeModule m_telescope;
    private final TopPivotModule m_topPivot;

    private final PinkArmKinematics m_kinematics;

    private final DataLog m_log;

    private final Mechanism2d m_mechanism2d;
    private final MechanismRoot2d m_mechanismRoot2d;
    private final MechanismLigament2d m_telescopeLigament;
    private final MechanismLigament2d m_feederLigament;

    private PinkArmState m_state;

    public PinkArmSubsystem(DataLog log) {
        m_log = log;

        m_bottomPivot = new BottomPivotModule(m_log);
        m_telescope = new TelescopeModule(m_log);
        m_topPivot = new TopPivotModule(m_log);

        m_mechanism2d = new Mechanism2d(3, 3);
        m_mechanismRoot2d = m_mechanism2d.getRoot("Bottom Pivot", 1.5, 1.5);
        m_telescopeLigament = m_mechanismRoot2d
                .append(new MechanismLigament2d("Telescope", 1, -90, 6, new Color8Bit(Color.kAqua)));
        m_feederLigament = m_telescopeLigament
                .append(new MechanismLigament2d("Top Pivot", 0.2, 0, 6, new Color8Bit(Color.kYellow)));

        m_kinematics = new PinkArmKinematics(0, 5);

        addChild("Bottom Pivot", m_bottomPivot);
        addChild("Telescope", m_telescope);
        addChild("Top Pivot", m_topPivot);
    }

    @Override
    public void periodic() {
        m_bottomPivot.periodic();
        m_telescope.periodic();
        m_topPivot.periodic();

        m_telescopeLigament.setAngle(m_bottomPivot.getAngle());
        m_telescopeLigament.setLength(m_telescope.getExtension());
        m_feederLigament.setAngle(m_topPivot.getAngle());

        SmartDashboard.putData("Pink Arm", m_mechanism2d);
    }

    @Override
    public void simulationPeriodic() {
        m_bottomPivot.simulate();
        m_telescope.simulate();
        m_topPivot.simulate();
    }

    public PinkArmState getState() {
        return new PinkArmState(m_telescope.getExtension(), new Rotation2d(Math.toRadians(m_bottomPivot.getAngle())), new Rotation2d(Math.toRadians(m_topPivot.getAngle())));
    }

    public void setDesiredState(PinkArmState state) {
        System.out.println(state.getBottomAngle().getDegrees());
        m_bottomPivot.setAngle(state.getBottomAngle().getDegrees(), m_telescope.getExtension());
        m_telescope.setExtension(state.getExtension());
        m_topPivot.setAngle(state.getTopAngle().getDegrees());
    }

    public void setPosition(double x, double y) {
        m_state = m_kinematics.toPinkArmState(x, y);

        setDesiredState(m_state);
    }

    public void setDesiredVelocities(double desiredBottomSpeed,
            double desiredExtensionSpeed, double desiredTopSpeed) {
        m_bottomPivot.setVelocity(desiredBottomSpeed, m_telescope.getExtension());
        m_telescope.setVelocity(desiredExtensionSpeed);
        m_topPivot.setVelocity(desiredTopSpeed);
    }

}
