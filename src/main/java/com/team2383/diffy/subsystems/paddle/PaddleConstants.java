package com.team2383.diffy.subsystems.paddle;

import edu.wpi.first.math.controller.PIDController;

public final class PaddleConstants {
    public static final int ID = 9;
    public static final PIDController PID_CONTROLLER = new PIDController(0.5, 0, 0);
    public static int kMaxCurrent = 20;
    public static double encoderOffset = 8.186;

}
