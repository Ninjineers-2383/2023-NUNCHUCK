// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Note: Translations calculated with the formula graphed here
 * https://www.desmos.com/calculator/yoi36dcace
 */
public final class Constants {
    public static final String kRIOBus = "rio";
    public static final String kCANivoreBus = "rio"; // TODO: Change to canivore when available

    public static final class DriveConstants {
        public static final double kMaxSpeed = 4; // meters per second
        public static final double kMaxAngularSpeed = 4 * Math.PI; // radians per second
    }

    public static final class ModuleConstants {
        public final static double kDriveMaxVoltage = 10.0;
        public final static double kMaxCurrent = 40.0;

        public final static double kPModuleTurningController = 0.8 / Math.PI;

        public final static double kPModuleDriveController = 0.8;
        public final static double kIModuleDriveController = 0.003;
        public final static double kDModuleDriveController = 0.02;

        public final static double kMaxTurnOutput = 0.5;

        public final static double ks = 0.45;
        public final static double kv = 0.1;
        public final static double ka = 0.7;

        public final static double kDriveGearRatio = (9 / 60.0) * (20 / 84.0) * (64 / 16.0); // 1/7
        public final static double kTurnGearRatio = 1 / 28.0;

        public final static double kDriveWheelDiameterMeters = 0.1016; // 4 inches

        public final static double kMaxAngularVelocity = Math.PI * 100;
        public final static double kMaxAngularAcceleration = Math.PI * 2 * 100;

        public final static double kTrackWidthMeters = 0.6173724;
    }

    public static final class FrontLeftModule {
        public final static int kTopMotorID = 20;
        public final static int kBottomMotorID = 21;
        public final static int kEncoderPortA = 0;
        public final static int kEncoderPortB = 1;
        public final static int kEncoderPortAbs = 2;
        public final static String name = "frontLeft";
        public final static Translation2d translation = new Translation2d(
                (Math.sqrt(3) * ModuleConstants.kTrackWidthMeters) / 4,
                ModuleConstants.kTrackWidthMeters / 2);
        public final static Rotation2d staticAngle = Rotation2d.fromDegrees(-45);
        public final static Rotation2d mountAngle = Rotation2d.fromDegrees(60);
    }

    public static final class FrontRightModule {
        public final static int kTopMotorID = 22;
        public final static int kBottomMotorID = 23;
        public final static int kEncoderPortA = 3;
        public final static int kEncoderPortB = 4;
        public final static int kEncoderPortAbs = 5;
        public final static String name = "frontRight";
        public final static Translation2d translation = new Translation2d(
                (Math.sqrt(3) * ModuleConstants.kTrackWidthMeters) / 4,
                -ModuleConstants.kTrackWidthMeters / 2);
        public final static Rotation2d staticAngle = Rotation2d.fromDegrees(45);
        public final static Rotation2d mountAngle = Rotation2d.fromDegrees(-60);
    }

    public static final class RearModule {
        public final static int kTopMotorID = 24;
        public final static int kBottomMotorID = 25;
        public final static int kEncoderPortA = 7;
        public final static int kEncoderPortB = 8;
        public final static int kEncoderPortAbs = 9;
        public final static String name = "rear";
        public final static Translation2d translation = new Translation2d(
                -(Math.sqrt(3) * ModuleConstants.kTrackWidthMeters) / 4,
                0);
        public final static Rotation2d staticAngle = Rotation2d.fromDegrees(90);
        public final static Rotation2d mountAngle = Rotation2d.fromDegrees(0);
    }
}
