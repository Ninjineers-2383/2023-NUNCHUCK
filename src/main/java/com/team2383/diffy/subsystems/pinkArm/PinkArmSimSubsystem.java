package com.team2383.diffy.subsystems.pinkArm;

import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PinkArmSimSubsystem extends SubsystemBase {
    private final PivotSubsystem m_pivot;
    private final TelescopeSubsystem m_telescope;
    private final WristSubsystem m_wrist;

    private final Mechanism2d m_mechanism2d;
    private final MechanismRoot2d m_mechanismRoot2d;
    private final MechanismLigament2d m_telescopeLigament;
    private final MechanismLigament2d m_feederLigament;

    public PinkArmSimSubsystem(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist) {
        m_pivot = pivot;
        m_telescope = telescope;
        m_wrist = wrist;

        m_mechanism2d = new Mechanism2d(10, 10);
        m_mechanismRoot2d = m_mechanism2d.getRoot("Bottom Pivot", 5, 5);
        m_telescopeLigament = m_mechanismRoot2d
                .append(new MechanismLigament2d("Telescope", 1, -90, 6, new Color8Bit(Color.kAqua)));
        m_feederLigament = m_telescopeLigament
                .append(new MechanismLigament2d("Top Pivot", 0.2, 0, 6, new Color8Bit(Color.kYellow)));
    }

    @Override
    public void periodic() {
        m_telescopeLigament.setAngle(m_pivot.getAngleDegrees() - 90);
        m_telescopeLigament.setLength((m_telescope.getExtensionInches() >= 0 ? m_telescope.getExtensionInches() : 0) + 1);
        m_feederLigament.setAngle(m_wrist.getAngleDegrees() + 180);

        SmartDashboard.putData("Pink Arm", m_mechanism2d);
    }
}
