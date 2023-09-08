package com.team2383.nunchuck.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean[] connected = {};
        public double[] x = {};
        public double[] y = {};
        public double[] z = {};

        public double[] roll = {};
        public double[] pitch = {};
        public double[] yaw = {};

        public double[] timestampSeconds = {};

        public void clear() {
            connected = new boolean[] {};
            x = new double[] {};
            y = new double[] {};
            z = new double[] {};

            roll = new double[] {};
            pitch = new double[] {};
            yaw = new double[] {};

            timestampSeconds = new double[] {};
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {
    }
}
