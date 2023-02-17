package com.team2383.diffy.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class DriveConstants {
    public static final double kMaxVelocity = 4; // meters per second

    public final static double kTrackWidthMeters = 0.6173724;
    public final static double kDriveMaxVoltage = 10.0;
    public final static double kMaxCurrent = 40.0;

    public final static double kDriveGearRatio = (9 / 60.0) * (20 / 84.0) * (64 / 16.0); // 1/7
    public final static double kTurnGearRatio = 1 / 28.0;

    public final static double kDriveWheelDiameterMeters = 0.1016; // 4 inches
    
    public final static double kMaxAngularVelocity = Math.PI * 100;
    public final static double kMaxAngularAcceleration = Math.PI * 2 * 100;

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
            0.19054, 0.017289, 0.00041192,
            20, 21,
            0, 1, 2,
            "frontLeft",
            new Translation2d(
                    (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(60));

    public final static ModuleConstants frontRightConstants = new ModuleConstants(
            0.19054, 0.017289, 0.00041192,
            22, 23,
            3, 4, 5,
            "frontRight",
            new Translation2d(
                    (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    -DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(-60));

    public final static ModuleConstants rearConstants = new ModuleConstants(
            0.19054, 0.017289, 0.00041192,
            24, 25,
            7, 8, 9,
            "rear",
            new Translation2d(
                    -(Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    0),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(180));

}
