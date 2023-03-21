package com.team2383.diffy.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class DriveConstants {
    public static final double kMaxVelocity = 5; // meters per second

    public final static double kTrackWidthMeters = 0.6173724;
    public final static double kDriveMaxVoltage = 9.0;
    public final static double kMaxCurrent = 30.0;

    public final static double kTurnGearRatio = (10 / 58.0) * (20 / 84.0);
    public final static double kDriveGearRatio = kTurnGearRatio * (64 / 16.0); // 1/7

    public final static double kDriveWheelDiameterMeters = 0.1016; // 4 inches

    public final static double kMaxAngularVelocity = Math.PI * 20;
    public final static double kMaxAngularAcceleration = Math.PI * 2 * 100;

    public final static PIDController HEADING_CONTROLLER = new PIDController(1, 0, 0);

    public static final class ModuleConstants {
        public final double kS;
        public final double kV;
        public final double kA;

        public final int kTopMotorID;
        public final int kBottomMotorID;

        public final int kEncoderPortA;
        public final int kEncoderPortB;
        public final int kEncoderPortAbs;

        public final String name;
        public final Translation2d translation;
        public final Rotation2d staticAngle;
        public final Rotation2d mountAngle;

        public ModuleConstants(double kS, double kV, double kA,
                int kTopMotorID, int kBottomMotorID,
                int kEncoderPortA, int kEncoderPortB, int kEncoderPortAbs,
                String name, Translation2d translation,
                Rotation2d staticAngle, Rotation2d mountAngle) {

            this.kS = kS;
            this.kV = kV;
            this.kA = kA;

            this.kTopMotorID = kTopMotorID;
            this.kBottomMotorID = kBottomMotorID;

            this.kEncoderPortA = kEncoderPortA;
            this.kEncoderPortB = kEncoderPortB;
            this.kEncoderPortAbs = kEncoderPortAbs;

            this.name = name;
            this.translation = translation;
            this.staticAngle = staticAngle;
            this.mountAngle = mountAngle;
        }
    }

    public final static ModuleConstants frontLeftConstants = new ModuleConstants(
            0.12042,
            0.018865,
            0.00092008,
            20, 21,
            0, 1, 2,
            "frontLeft",
            new Translation2d(
                    (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(60));

    public final static ModuleConstants frontRightConstants = new ModuleConstants(
            0.2029,
            0.018601,
            0.00038568,
            22, 23,
            3, 4, 5,
            "frontRight",
            new Translation2d(
                    (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    -DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(-60));

    public final static ModuleConstants rearConstants = new ModuleConstants(
            0.23873,
            0.018696,
            0.00033035,
            24, 25,
            7, 8, 9,
            "rear",
            new Translation2d(
                    -(Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    0),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(180));

}
