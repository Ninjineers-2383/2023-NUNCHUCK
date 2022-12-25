package frc.robot.helpers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Feedforward implements Sendable {
    private double ks;
    private double ka;
    private double kv;

    public Feedforward(double ks, double kv, double ka) {
        this.ka = ka;
        this.ks = ks;
        this.kv = kv;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    public double getKs() {
        return ks;
    }

    public double getKv() {
        return kv;
    }

    public double getKa() {
        return ka;
    }

    public void setKs(double ks) {
        this.ks = ks;
    }

    public void setKv(double kv) {
        this.kv = kv;
    }

    public void setKa(double ka) {
        this.ka = ka;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FFController");
        builder.addDoubleProperty("s", this::getKs, this::setKs);
        builder.addDoubleProperty("v", this::getKv, this::setKv);
        builder.addDoubleProperty("a", this::getKa, this::setKa);
    }
}
