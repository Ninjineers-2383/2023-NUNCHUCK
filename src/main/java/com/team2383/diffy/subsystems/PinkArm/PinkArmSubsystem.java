package com.team2383.diffy.subsystems.PinkArm;

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
    private final MainPivotModule m_bottomPivot;
    private final TelescopeModule m_telescope;
    private final WristPivotModule m_wristPivot;

    private final PinkArmKinematics m_kinematics;

    private final DataLog m_log;

    private final Mechanism2d m_mechanism2d;
    private final MechanismRoot2d m_mechanismRoot2d;
    private final MechanismLigament2d m_telescopeLigament;
    private final MechanismLigament2d m_feederLigament;

    private PinkArmState m_state;

    public PinkArmSubsystem(DataLog log) {
        m_log = log;

        m_bottomPivot = new MainPivotModule(m_log);
        m_telescope = new TelescopeModule(m_log);
        m_wristPivot = new WristPivotModule(m_log);

        m_mechanism2d = new Mechanism2d(10, 10);
        m_mechanismRoot2d = m_mechanism2d.getRoot("Bottom Pivot", 5, 5);
        m_telescopeLigament = m_mechanismRoot2d
                .append(new MechanismLigament2d("Telescope", 1, -90, 6, new Color8Bit(Color.kAqua)));
        m_feederLigament = m_telescopeLigament
                .append(new MechanismLigament2d("Top Pivot", 0.2, 0, 6, new Color8Bit(Color.kYellow)));

        m_kinematics = new PinkArmKinematics(1);

        addChild("Bottom Pivot", m_bottomPivot);
        addChild("Telescope", m_telescope);
        addChild("Top Pivot", m_wristPivot);
    }

    @Override
    public void periodic() {
        m_bottomPivot.periodic();
        m_telescope.periodic();
        m_wristPivot.periodic();

        double extension = m_telescope.getExtension();

        m_telescopeLigament.setAngle(m_bottomPivot.getAngleDegrees() - 90);
        m_telescopeLigament.setLength((extension >= 0 ? extension : 0) + 1);
        m_feederLigament.setAngle(m_wristPivot.getAngleDegrees() + 180);

        SmartDashboard.putData("Pink Arm", m_mechanism2d);

        if (m_state == null)
            return;

        PinkArmState cState = getState();

        if (cState.getExtension() > 0.1 && Math.signum(m_state.getBottomAngle().getDegrees()) != Math
                .signum(cState.getBottomAngle().getDegrees())) {
            m_wristPivot.setAngle(0, cState.getBottomAngle().getRadians());
            if (Math.abs(cState.getTopAngle().getDegrees()) < 2) {
                m_telescope.setExtension(0);
            }
        } else {
            if (Math.abs(cState.getBottomAngle().getDegrees() - m_state.getBottomAngle().getDegrees()) > 1) {
                m_bottomPivot.setAngle(m_state.getBottomAngle().getRadians(),
                        m_telescope.getExtension());
            } else if (Math.abs(cState.getTopAngle().getDegrees() - m_state.getTopAngle().getDegrees()) > 1) {
                m_wristPivot.setAngle(m_state.getTopAngle().getRadians(), cState.getBottomAngle().getRadians());
            } else if (Math.abs(cState.getExtension() - m_state.getExtension()) > 1) {
                m_telescope.setExtension(m_state.getExtension());
            }
        }

    }

    @Override
    public void simulationPeriodic() {
        m_bottomPivot.simulate();
        m_telescope.simulate();
        m_wristPivot.simulate();
    }

    public PinkArmState getState() {
        return new PinkArmState(m_telescope.getExtension(),
                new Rotation2d(m_bottomPivot.getAngleRadians()),
                new Rotation2d(m_wristPivot.getAngleRadians()));
    }

    public void setDesiredState(PinkArmState state) {
        m_state = state;
    }

    public void setPosition(double x, double y, double topAngle) {
        m_state = m_kinematics.toPinkArmState(x, y, topAngle);
        System.out.println("Bottom Angle: " + m_state.getBottomAngle());
        System.out.println("Top Angle: " + m_state.getTopAngle());
        System.out.println("Extension: " + m_state.getExtension());

        setDesiredState(m_state);
    }

    public void setDesiredVelocities(double desiredBottomSpeed,
            double desiredExtensionSpeed, double desiredTopSpeed) {
        m_bottomPivot.setVelocity(desiredBottomSpeed, m_telescope.getExtension());
        m_telescope.setVelocity(desiredExtensionSpeed);
        m_wristPivot.setVelocity(desiredTopSpeed, m_bottomPivot.getAngleRadians());
    }

    public boolean isAtPosition() {
        PinkArmState desiredState = m_state;
        PinkArmState currentState = getState();

        return Math.abs(desiredState.getBottomAngle().getDegrees()
                - currentState.getBottomAngle().getDegrees()) < 1
                && Math.abs(desiredState.getTopAngle().getDegrees()
                        - currentState.getTopAngle().getDegrees()) < 1
                && Math.abs(desiredState.getExtension() - currentState.getExtension()) < 1;
    }

}
