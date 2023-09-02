package com.team2383.nunchuck.subsystems.pinkArm.wrist;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.math.Clip;
import com.team2383.nunchuck.helpers.TrapezoidalSubsystemBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristSubsystem extends TrapezoidalSubsystemBase {
    private final WristIO m_io;
    private final WristIOInputsAutoLogged m_inputs = new WristIOInputsAutoLogged();

    double m_simVelocity = 0;

    private Supplier<Rotation2d> m_pivotAngle;

    public WristSubsystem(WristIO io, Supplier<Rotation2d> pivotAngle) {
        super("Wrist", WristConstants.TRAPEZOIDAL_CONSTRAINTS, WristConstants.SIMULATION_SUBSYSTEM,
                WristConstants.POSITION_THRESHOLD.getRadians());
        
        m_io = io;

        m_pivotAngle = pivotAngle;

        SmartDashboard.putData("Wrist FF", WristConstants.FEEDFORWARD_CONTROLLER);
        SmartDashboard.putData("Wrist PID", WristConstants.PID_CONTROLLER);
    }

    @Override
    public void periodic() {
        super.periodic();
        m_io.updateInputs(m_inputs);

        Logger.getInstance().processInputs("Wrist", m_inputs);
    }

    public void setPivotAngle(Supplier<Rotation2d> pivotAngle) {
        m_pivotAngle = pivotAngle;
    }

    public void setGoal(Rotation2d desiredAngle) {
        // safety for upper bounds
        double adjustedAngle = Clip.clip(WristConstants.kLowerBound.getRadians(), desiredAngle.getRadians(),
                WristConstants.kUpperBound.getRadians());
        super.setGoal(new TrapezoidProfile.State(adjustedAngle, 0));
    }

    public void setVelocity(Rotation2d velocity) {
        super.setVelocity(velocity.getRadians());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_inputs.angle);
    }

    @Override
    public void setVoltage(double voltage) {
        m_io.setVoltage(voltage);
    }

    @Override
    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getAngle().getRadians(), getVelocity().getRadians());
    }

    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(m_inputs.velocity);
    }

    private double getAbsoluteAngleRadians() {
        return -getAngle().getRadians() + 2.36
                + (m_pivotAngle != null ? m_pivotAngle.get().getRadians() - Math.PI / 2 : 0);
    }

    @Override
    protected double calculateVoltage(double velocity, double position) {
        double voltage = -WristConstants.PID_CONTROLLER.calculate(getAngle().getRadians(), position);
        voltage -= WristConstants.FEEDFORWARD_CONTROLLER.calculate(
                getAbsoluteAngleRadians(),
                velocity);
        return voltage;
    }
}
