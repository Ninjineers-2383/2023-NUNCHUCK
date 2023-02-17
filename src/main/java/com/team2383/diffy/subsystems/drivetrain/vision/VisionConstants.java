package com.team2383.diffy.subsystems.Drivetrain.vision;

import edu.wpi.first.math.VecBuilder;
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
                new PhotonCameraData("Microsoft_LifeCam_HD-3000",
                        new Transform3d(
                                new Translation3d(Units.inchesToMeters(-4), Units.inchesToMeters(-5.5),
                                        Units.inchesToMeters(1.0)),
                                new Rotation3d(VecBuilder.fill(0, 1, 0), Math.PI))),
        };

}