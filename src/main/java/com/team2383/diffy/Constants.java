// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.diffy;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Note: Translations calculated with the formula graphed here
 * https://www.desmos.com/calculator/jj9i5bdjhe
 * <p>
 * The circle function is used in
 * {@link com.team2383.diffy.commands.JoystickDriveCommand#getCenterOfRotation
 * JoystickDriveCommand}
 */

public final class Constants {
    public static final String kRIOBus = "rio";
    public static final String kCANivoreBus = "Drive";

    public static final class VisionConstants {
        public static final PhotonCameraData[] kPhotonCameras = new PhotonCameraData[] {
                new PhotonCameraData("Microsoft_LifeCam_HD-3000",
                        new Transform3d(
                                new Translation3d(-4, -5.5, 1.0),
                                new Rotation3d(VecBuilder.fill(0, 1, 0), Math.PI))),
        };
    }

    public static final class PhotonCameraData {
        public final String name;
        public final Transform3d transform;

        public PhotonCameraData(String name, Transform3d transform) {
            this.name = name;
            this.transform = transform;
        }
    }

    public static final class DriveConstants {
        public static final double kMaxVelocity = 4; // meters per second
        public static final double kMaxAngularVelocity = 4 * Math.PI; // radians per second

        public final static double kTrackWidthMeters = 0.6173724;

        public final static ModuleConstants frontLeftConstants = new ModuleConstants(
                0.59054, 0.017289, 0.00041192,
                20, 21,
                0, 1, 2,
                "frontLeft",
                new Translation2d(
                        (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                        DriveConstants.kTrackWidthMeters / 2),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(60));

        public final static ModuleConstants frontRightConstants = new ModuleConstants(
                0.59054, 0.017289, 0.00041192,
                22, 23,
                3, 4, 5,
                "frontRight",
                new Translation2d(
                        (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                        -DriveConstants.kTrackWidthMeters / 2),
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(-60));

        public final static ModuleConstants rearConstants = new ModuleConstants(
                0.59054, 0.017289, 0.00041192,
                24, 25,
                7, 8, 9,
                "rear",
                new Translation2d(
                        -(Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                        0),
                Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(180));

    }

    public static final class GlobalModuleConstants {
        public final static double kDriveMaxVoltage = 10.0;
        public final static double kMaxCurrent = 40.0;

        public final static double kDriveGearRatio = (9 / 60.0) * (20 / 84.0) * (64 / 16.0); // 1/7
        public final static double kTurnGearRatio = 1 / 28.0;

        public final static double kDriveWheelDiameterMeters = 0.1016; // 4 inches

        public final static double kMaxAngularVelocity = Math.PI * 100;
        public final static double kMaxAngularAcceleration = Math.PI * 2 * 100;
    }

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

    public static final class TelescopeConstants {
        // FF Values
        // TODO: Tune these values
        public static final double kS = 0.01;
        public static final double kV = 0.01;
        public static final double kA = 0.009;

        public static final double kUpperBound = 10;
        public static final double kLowerBound = 0;

        public static final double kMaxCurrent = 40.0;

        public static final int kExtensionLeftID = 1;
        public static final int kExtensionRightID = 2;

        public static final double kRadPerSecToInchesPerSec =  19.25 / 192.0;
    
    }

    public static final class DickConstants {
        public static final int port = 8;
    }

    public static final class BottomPivotConstants {
        public static final double kS = 0.01;
        public static final double kV = 0.1;
        public static final double kA = 0.001;

        public static final double kUpperBound = 90;
        public static final double kLowerBound = 0;
        
        public static final double pivotLength = 0.5;
        public static final double gravity = 9.8;
        public static final double armMass = 10;

        public static final double kMaxCurrent = 40.0;

        public static final int kBottomMotorLeftId = 29;
        public static final int kBottomMotorRightId = 30;

        public static final int kEncoderPortA = 10;
        public static final int kEncoderPortB = 11;
        public static final int kEncoderPortAbs = 12;

        public static final double kgb = 1 / 112.5;
    }

    public static final class TopPivotConstants {
        // FF Values
        // TODO: Tune these values
        public static final double kS = 0.01;
        public static final double kV = 0.01465;
        public static final double kA = 0.001;
        public static final double kG = 0.01;

        public static final double kUpperBound = 10;
        public static final double kLowerBound = 0;

        public static final double kMaxCurrent = 40.0;
        public static final int kMotorID = 31;

        public static final int kEncoderPortA = 13;
        public static final int kEncoderPortB = 14;
        public static final int kEncoderPortAbs = 15;

        public static final double kgt = 1 / 218.7;
    }

    public static final class PinkArmConstants {
        public static final double BottomPivotVelocity = .1;
        public static final double TopPivotVelocity = .1;
        public static final double TelescopeVelocity = .1;
    }

    public static final class FeederConstants {
        // TODO: Tune SS control values

        public static final double kV = 0.01;
        public static final double kA = 0.001;

        public static final int kTopMotorID = 26;
        public static final int kBottomMotorID = 27;
        public static final int kClawMotorID = 28;
    }
}
