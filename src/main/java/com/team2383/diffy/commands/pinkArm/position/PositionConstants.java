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
            new Rotation2d(0.415),
            1,
            new Rotation2d(-2.375029));

    public static final PinkPositions FEED_CONE_POS = new PinkPositions(
            "Feed Cone",
            Rotation2d.fromDegrees(35),
            16,
            Rotation2d.fromDegrees(-120));

    public static final PinkPositions FEED_CONE_CHUTE = new PinkPositions(
            "Feed Chute",
            new Rotation2d(0.352),
            0,
            new Rotation2d(-0.8145));

    public static final PinkPositions FEED_PADDLE_POS_INIT = new PinkPositions(
            "Transfer",
            Rotation2d.fromDegrees(-41),
            0.31,
            Rotation2d.fromDegrees(-45));

    public static final PinkPositions TRAVEL_POS = new PinkPositions(
            "Travel Position",
            Rotation2d.fromDegrees(-18),
            0,
            Rotation2d.fromDegrees(0));

    public static final PinkPositions FEED_CONE_UPRIGHT = new PinkPositions(
            "Feed Cone Upright",
            Rotation2d.fromRadians(0.8),
            1,
            Rotation2d.fromRadians(-2.5));

    // public static final PinkPositions TRANSFER_POS = new PinkPositions(
    // "Transfer Position",
    // Rotation2d.fromDegrees(-41),
    // 0,
    // Rotation2d.fromDegrees(0));

    public static final PinkPositions FEED_UPRIGHT_CONE = new PinkPositions(
            "Feed Upright",
            Rotation2d.fromDegrees(60),
            2,
            Rotation2d.fromDegrees(108));

    public static final PinkPositions LOW_SCORE_BACK = new PinkPositions(
            "Low Score",
            Rotation2d.fromDegrees(-62),
            3,
            Rotation2d.fromDegrees(-125));

    public static final PinkPositions MID_SCORE_BACK = new PinkPositions(
            "Mid Score Back",
            Rotation2d.fromDegrees(-125),
            2,
            Rotation2d.fromDegrees(-100));

    public static final PinkPositions HIGH_SCORE_BACK = new PinkPositions(
            "High Score Back",
            Rotation2d.fromDegrees(-125),
            20,
            Rotation2d.fromDegrees(-120));

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

    public static final PinkPositions FEED_INTERNAL = new PinkPositions(
            "Feed Internal",
            Rotation2d.fromRadians(0.32),
            4.8,
            Rotation2d.fromRadians(-2.05));
}
