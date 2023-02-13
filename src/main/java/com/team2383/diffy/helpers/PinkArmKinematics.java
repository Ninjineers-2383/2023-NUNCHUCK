package com.team2383.diffy.helpers;

import edu.wpi.first.math.geometry.Rotation2d;

public class PinkArmKinematics {
    private final double m_extensionZeroLength;

    public PinkArmKinematics(double extensionZeroLength) {
        m_extensionZeroLength = extensionZeroLength;
    }

    public PinkArmState toPinkArmState(double x, double y, double topAngle) {
        double extension = Math.sqrt(x * x + y * y) - m_extensionZeroLength;
        double bottom = Math.atan2(y, x) + Math.PI / 2;
        double top = topAngle - bottom - Math.PI / 2;

        return new PinkArmState(extension, new Rotation2d(bottom), new Rotation2d(top));
    }
}
