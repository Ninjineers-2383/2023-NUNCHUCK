package com.team2383.diffy.subsystems.pinkArm.wrist;

public final class WristConstants {
    // FF Values
    // TODO: Tune these values
    public static final double kS = 0.21143;
    public static final double kV = 2.091;
    public static final double kA = 0.10864;
    public static final double kP = 1;

    public static final double kG = 1.4;

    public static final double encoderOffset = .52;

    public static final double kUpperBound = 180;
    public static final double kLowerBound = -180;

    public static final double kMaxCurrent = 40.0;

    public static final int kMotorID = 8;

    public static final int kEncoderPortAbs = 6;

    public static final double kgt = 1 / 218.7;

}