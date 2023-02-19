package com.team2383.diffy.subsystems.pinkArm.pivot;

import edu.wpi.first.math.util.Units;

public  final class PivotConstants {
    public static final double kS = 0.48118;
    public static final double kV = 0.01;
    public static final double kA = 0.232;
    public static final double kP = 0.005;
    public static final double kD = 0.01;
    public static final double kG = 0.5;

    public static final double PivotVelocity = .1;

    public static final double kUpperBound = Units.degreesToRadians(170);
    public static final double kLowerBound = Units.degreesToRadians(-170);

    public static final double kUpperSafety = Units.degreesToRadians(30);
    public static final double kLowerSafety = Units.degreesToRadians(-30);
    public static final double extensionSafety = 1;

    public static final double pivotLength = 0.5;
    public static final double gravity = 9.8;
    public static final double armMass = 10;

    public static final double kMaxCurrent = 40.0;

    public static final int kBottomMotorLeftId = 2;
    public static final int kBottomMotorRightId = 3;

    public static final int kEncoderPortAbs = 6;
    public static final double encoderOffset = 13.0/360;

    public static final double kgb = 1 / 112.5;
}