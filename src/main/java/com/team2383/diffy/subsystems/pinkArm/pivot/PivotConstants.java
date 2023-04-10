package com.team2383.diffy.subsystems.pinkArm.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class PivotConstants {
    public static final int BOTTOM_MOTOR_LEFT_ID = 2;
    public static final int BOTTOM_MOTOR_RIGHT_ID = 3;
    public static final int ABS_ENCODER_ID = 6;

    public static final int MAX_CURRENT = 40;
    public static final double ENCODER_OFFSET = 3.365 / (2 * Math.PI);

    public static final Rotation2d POSITION_THRESHOLD = Rotation2d.fromDegrees(10);

    public static final Rotation2d UPPER_BOUND = Rotation2d.fromDegrees(170);
    public static final Rotation2d LOWER_BOUND = Rotation2d.fromDegrees(-170);

    public static final Rotation2d UPPER_SAFETY = Rotation2d.fromDegrees(0);
    public static final Rotation2d LOWER_SAFETY = Rotation2d.fromDegrees(-0);

    public static final double kG = 0.98;

    public static final double kGOFFSET = 25;

    public static final PIDController PID_CONTROLLER = new PIDController(14, 0, 0);
    public static final SimpleMotorFeedforward FEEDFORWARD_CONTROLLER = new SimpleMotorFeedforward(0.5, 1.65, // kv:
                                                                                                              // 1.65
            0.001);

    public static final Rotation2d VELOCITY_CONVERSION_FACTOR = Rotation2d.fromDegrees(360 * 60);

    // Values will be in radians and seconds
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(5,
            4);

    public static final LinearSystem<N1, N1, N1> SIMULATION_SUBSYSTEM = LinearSystemId.identifyVelocitySystem(
            PivotConstants.FEEDFORWARD_CONTROLLER.kv,
            PivotConstants.FEEDFORWARD_CONTROLLER.ka);
}