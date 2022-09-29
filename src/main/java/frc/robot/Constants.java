// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Note: Translations calculated with the formula graphed here
 * https://www.desmos.com/calculator/yoi36dcace
 */
public final class Constants {
    public static final String kRIOBus = "rio";
    public static final String kCANivoreBus = "rio"; // TODO: Change to canivore when available

    public static final class DriveConstants {
        public static final double kMaxSpeed = 5.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    }

    public static final class ModuleConstants {
        public final static double kDriveMaxVoltage = 10.0;
        public final static double kMaxCurrent = 40.0;

        // public final static double kPModuleTurningController = 0 / Math.PI;
        public final static double kPModuleTurningController = 0.8 / Math.PI;

        public final static double kPModuleDriveController = 0.025;
        // public final static double kPModuleDriveController = 0.01;

        public final static double kMaxTurnOutput = 0.5;

        public final static double ks = 0.57219;
        public final static double kv = 2.184;
        public final static double ka = 0.067772;
        // public final static double ks = 0.2;
        // public final static double kv = 0.012;
        // public final static double ka = 0.1;

        public final static double kDriveGearRatio = 6.4;
        public final static double kTurnGearRatio = 28;

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
        public final static Translation2d translation = new Translation2d(-ModuleConstants.kTrackWidthMeters / 2,
                (Math.sqrt(3) * ModuleConstants.kTrackWidthMeters) / 4);
    }

    public static final class FrontRightModule {
        public final static int kTopMotorID = 22;
        public final static int kBottomMotorID = 23;
        public final static int kEncoderPortA = 6;
        public final static int kEncoderPortB = 7;
        public final static int kEncoderPortAbs = 8;
        public final static String name = "frontRight";
        public final static Translation2d translation = new Translation2d(ModuleConstants.kTrackWidthMeters / 2,
                (Math.sqrt(3) * ModuleConstants.kTrackWidthMeters) / 4);
    }

    public static final class RearModule {
        public final static int kTopMotorID = 24;
        public final static int kBottomMotorID = 25;
        public final static int kEncoderPortA = 3;
        public final static int kEncoderPortB = 4;
        public final static int kEncoderPortAbs = 5;
        public final static String name = "rear";
        public final static Translation2d translation = new Translation2d(0,
                -(Math.sqrt(3) * ModuleConstants.kTrackWidthMeters) / 4);
    }
}
