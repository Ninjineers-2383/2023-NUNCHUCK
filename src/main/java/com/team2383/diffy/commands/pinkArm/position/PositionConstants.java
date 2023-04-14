package com.team2383.diffy.commands.pinkArm.position;

import edu.wpi.first.math.geometry.Rotation2d;

public class PositionConstants {

    public static class PinkPositions {
        public final String name;
        public final Rotation2d pivot;
        public final double extension;
        public final Rotation2d wrist;

        public PinkPositions(String name, Rotation2d pivot, double extension, Rotation2d wrist) {
            this.name = name;
            this.pivot = pivot;
            this.extension = extension;
            this.wrist = wrist;
        }
    }

    public static final PinkPositions ZERO_POS = new PinkPositions(
            "Zero",
            Rotation2d.fromDegrees(0),
            0,
            Rotation2d.fromDegrees(0));

    public static final PinkPositions FEED_CUBE_POS = new PinkPositions(
            "Feed Cube",
            new Rotation2d(0.773),
            5.68,
            new Rotation2d(2.465));

    public static final PinkPositions FEED_CONE_POS = new PinkPositions(
            "Feed Cone",
            Rotation2d.fromRadians(0.558),
            8.771,
            Rotation2d.fromRadians(1.587));

    public static final PinkPositions FEED_UPRIGHT = new PinkPositions(
            "Feed Upright",
            Rotation2d.fromRadians(0.718),
            1.694,
            Rotation2d.fromRadians(2.258));

    public static final PinkPositions FEED_HIGH = new PinkPositions(
            "Feed High",
            Rotation2d.fromRadians(2.406),
            0.69,
            Rotation2d.fromRadians(3.783));

    public static final PinkPositions FEED_CHUTE = new PinkPositions(
            "Feed Chute",
            new Rotation2d(0.2188),
            0,
            new Rotation2d(-0.02));

    public static final PinkPositions TRAVEL_POS = new PinkPositions(
            "Travel Position",
            Rotation2d.fromDegrees(-18),
            0,
            Rotation2d.fromRadians(0));

    public static final PinkPositions LOW_SCORE_BACK = new PinkPositions(
            "Low Score",
            Rotation2d.fromRadians(-1.1),
            0.3,
            Rotation2d.fromRadians(1.88));

    public static final PinkPositions MID_SCORE_BACK = new PinkPositions(
            "Mid Score Back",
            Rotation2d.fromRadians(-1.94),
            0,
            Rotation2d.fromRadians(1.16));

    public static final PinkPositions HIGH_SCORE_BACK = new PinkPositions(
            "High Score Back",
            Rotation2d.fromRadians(-2.12),
            19,
            Rotation2d.fromRadians(1.4));

    public static final PinkPositions MID_SCORE_FRONT = new PinkPositions(
            "Mid Score Front",
            new Rotation2d(1.95),
            14.8,
            new Rotation2d(1.64));

    public static final PinkPositions HIGH_SCORE_FRONT = new PinkPositions(
            "High Score Front",
            new Rotation2d(2.11),
            22,
            new Rotation2d(2.2));
}
