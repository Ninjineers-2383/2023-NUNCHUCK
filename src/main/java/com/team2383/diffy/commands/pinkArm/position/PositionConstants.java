package com.team2383.diffy.commands.pinkArm.position;

import edu.wpi.first.math.geometry.Rotation2d;

public class PositionConstants {

    public static class PinkPositions {
        public final Rotation2d pivot;
        public final double extension;
        public final Rotation2d wrist;

        public PinkPositions(Rotation2d pivot, double extension, Rotation2d wrist) {
            this.pivot = pivot;
            this.extension = extension;
            this.wrist = wrist;
        }
    }

    public static final PinkPositions ZERO_POS = new PinkPositions(
            Rotation2d.fromDegrees(0),
            0,
            Rotation2d.fromDegrees(0));

    public static final PinkPositions FEED_CUBE_POS = new PinkPositions(
            new Rotation2d(0.415),
            1,
            new Rotation2d(-2.375029));

    public static final PinkPositions FEED_CONE_POS = new PinkPositions(
            Rotation2d.fromDegrees(35),
            15,
            Rotation2d.fromDegrees(-122));

    public static final PinkPositions FEED_PADDLE_POS = new PinkPositions(
            Rotation2d.fromRadians(-0.29),
            0,
            Rotation2d.fromRadians(-0.26));

    public static final PinkPositions TRAVEL_POS = new PinkPositions(
            Rotation2d.fromDegrees(-18),
            0,
            Rotation2d.fromDegrees(0));

    public static final PinkPositions FEED_UPRIGHT_CONE = new PinkPositions(
            Rotation2d.fromDegrees(60),
            2,
            Rotation2d.fromDegrees(108));

    public static final PinkPositions LOW_SCORE_POS = new PinkPositions(
            Rotation2d.fromDegrees(-60),
            2,
            Rotation2d.fromDegrees(-120));

    public static final PinkPositions MID_SCORE_POS = new PinkPositions(
            Rotation2d.fromDegrees(-128),
            0,
            Rotation2d.fromDegrees(-95));

    public static final PinkPositions HIGH_SCORE_POS = new PinkPositions(
            Rotation2d.fromDegrees(-125),
            20,
            Rotation2d.fromDegrees(-90));
}
