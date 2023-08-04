package com.team2383.nunchuck.subsystems.drivetrain.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

    // Description of the camera's relative position
    private static final Rotation3d CAM_PITCH = new Rotation3d(0, Units.degreesToRadians(20), 0);
    private static final Rotation3d CAM_YAW = new Rotation3d(0, 0, Units.degreesToRadians(45));
    private static final Rotation3d FRONT_YAW = new Rotation3d(0, 0, Units.degreesToRadians(180));

    private static final Translation3d CAM_X = new Translation3d(Units.inchesToMeters(3.5) / 2, 0, 0);
    private static final Translation3d CAM_Y = new Translation3d(0, 0.908 / 2, 0);
    private static final Translation3d CAM_Z = new Translation3d(0, 0, Units.inchesToMeters(45));

    public static final class PhotonCameraData {
        public final String name;
        public final Transform3d transform;

        public PhotonCameraData(String name, Transform3d transform) {
            this.name = name;
            this.transform = transform;
        }
    }

    public static final PhotonCameraData[] kPhotonCameras = new PhotonCameraData[] {
            new PhotonCameraData("Rear",
                    new Transform3d(
                            CAM_Z,
                            FRONT_YAW)),
    };

}