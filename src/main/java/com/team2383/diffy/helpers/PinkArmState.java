package com.team2383.diffy.helpers;

import edu.wpi.first.math.geometry.Rotation2d;

public class PinkArmState {
    private double m_extensionLength;

    private Rotation2d m_bottomAngle;
    private Rotation2d m_topAngle;

    public PinkArmState(double extensionLength, Rotation2d bottomAngle, Rotation2d topAngle) {
        m_extensionLength = extensionLength;
        m_bottomAngle = bottomAngle;
        m_topAngle = topAngle;

    }
}
