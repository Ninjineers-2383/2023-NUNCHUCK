package com.team2383.diffy.subsystems.pinkArm.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public  final class PivotConstants {
    public static final int BOTTOM_MOTOR_LEFT_ID = 2;
    public static final int BOTTOM_MOTOR_RIGHT_ID = 3;
    public static final int ABS_ENCODER_ID = 6;

    public static final int MAX_CURRENT = 40;
    public static final double ENCODER_OFFSET = 13.0/360;

    public static final Rotation2d UPPER_BOUND = Rotation2d.fromDegrees(170);
    public static final Rotation2d LOWER_BOUND = Rotation2d.fromDegrees(-170);

    public static final Rotation2d UPPER_SAFETY = Rotation2d.fromDegrees(30);
    public static final Rotation2d LOWER_SAFETY = Rotation2d.fromDegrees(-30);


    public static final PIDController PID_CONTROLLER
        = new PIDController(0.005, 0, 0.01);
    public static final ArmFeedforward FEEDFORWARD_CONTROLLER
        = new ArmFeedforward(0.48118, 0.5, 0.232, 0.01);

    public static final Rotation2d VELOCITY_CONVERSION_FACTOR = Rotation2d.fromDegrees(360 * 60);

    // Values will be in radians and seconds
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS
        = new TrapezoidProfile.Constraints(1, 0.1);

    public static final LinearSystem<N1, N1, N1> SIMULATION_SUBSYSTEM = LinearSystemId.identifyVelocitySystem(
            PivotConstants.FEEDFORWARD_CONTROLLER.kv,
            PivotConstants.FEEDFORWARD_CONTROLLER.ka);
}