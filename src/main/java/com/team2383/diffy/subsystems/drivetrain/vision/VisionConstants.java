package com.team2383.diffy.subsystems.drivetrain.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    public static final class PhotonCameraData {
        public final String name;
        public final Transform3d transform;

        public PhotonCameraData(String name, Transform3d transform) {
            this.name = name;
            this.transform = transform;
        }
    }

    public static final PhotonCameraData[] kPhotonCameras = new PhotonCameraData[] {
            new PhotonCameraData("Arducam_OV9281_Front_Left",
                    new Transform3d(
                            new Translation3d(Units.inchesToMeters(1.5), Units.inchesToMeters(10),
                                    Units.inchesToMeters(45.5)),
                            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-40)))),
            new PhotonCameraData("Arducam_OV9281_Front_Right",
                    new Transform3d(
                            new Translation3d(Units.inchesToMeters(1.5), Units.inchesToMeters(-10),
                                    Units.inchesToMeters(45.5)),
                            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(40)))),
            new PhotonCameraData("Arducam_OV9281_Rear_Left",
                    new Transform3d(
                            new Translation3d(Units.inchesToMeters(1.5), Units.inchesToMeters(10),
                                    Units.inchesToMeters(45.5)),
                            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-180 + 40)))),
            new PhotonCameraData("Arducam_OV9281_Rear_Right",
                    new Transform3d(
                            new Translation3d(Units.inchesToMeters(1.5), Units.inchesToMeters(-10),
                                    Units.inchesToMeters(45.5)),
                            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180 - 40)))),
    };

}