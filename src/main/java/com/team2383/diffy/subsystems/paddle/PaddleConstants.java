package com.team2383.diffy.subsystems.paddle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public final class PaddleConstants {
    public static final int DICK_ID = 9;
    public static final int BALLS_ID = 10;

    public static final double kP = 2; // Radians per second

    public static final Rotation2d maxVelocityOut = Rotation2d.fromDegrees(40);
    public static final Rotation2d maxVelocityIn = Rotation2d.fromDegrees(30);
    public static final PIDController PID_CONTROLLER = new PIDController(0.5, 0.01, 0);
    public static int kMaxCurrent = 20;
    public static int encoderOffset = 0;// 1451;

}
