package com.team2383.diffy.helpers;

import edu.wpi.first.math.geometry.Rotation2d;

public class PinkArmKinematics {
    private double m_topPivotLength;

    public PinkArmKinematics(double extensionInitLength, double topPivotLength) {
        m_topPivotLength = topPivotLength;

    }

    public PinkArmState toPinkArmState(double x, double y, double topAngle) {
        double extension = calculateExtension(x, y, Math.toRadians(topAngle));
        System.out.println("Extension: " + extension);
        double bottomAngle = calculateBottomAngle(x, y, Math.toRadians(topAngle));
        return new PinkArmState(extension, new Rotation2d(bottomAngle), new Rotation2d(Math.toRadians(topAngle)));
    }

    public double calculateExtension(double x, double y, double topAngle) {
        double magnitude = Math.sqrt(x * x + y * y);
        double extension = (magnitude
                * Math.sin((Math.PI - topAngle - Math.asin((m_topPivotLength * Math.sin(topAngle))) / (2 * magnitude))))
                / Math.sin(topAngle);
        return extension;
    }

    public double calculateBottomAngle(double x, double y, double topAngle) {
        double magnitude = Math.sqrt(x * x + y * y);
        double bottomAngle = Math.atan2(y, x) + Math.asin((m_topPivotLength * Math.sin(topAngle)) / (2 * magnitude));
        return bottomAngle;
    }

}
