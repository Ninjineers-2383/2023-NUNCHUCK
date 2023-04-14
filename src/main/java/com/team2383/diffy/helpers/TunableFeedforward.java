package com.team2383.diffy.helpers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TunableFeedforward implements Sendable {
    public double ks;
    public double kv;
    public double ka;

    public TunableFeedforward(double kS, double kV, double kA) {
        this.ks = kS;
        this.kv = kV;
        this.ka = kA;
    }

    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TunableFeedforward");
        builder.addDoubleProperty("kS", () -> ks, (value) -> ks = value);
        builder.addDoubleProperty("kV", () -> kv, (value) -> kv = value);
        builder.addDoubleProperty("kA", () -> ka, (value) -> ka = value);
    }
}
