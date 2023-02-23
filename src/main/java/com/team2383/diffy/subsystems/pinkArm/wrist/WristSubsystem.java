package com.team2383.diffy.subsystems.pinkArm.wrist;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2383.diffy.Robot;
import com.team2383.diffy.helpers.AngularVelocityWrapper;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.TrapezoidalSubsystemBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;

public class WristSubsystem extends TrapezoidalSubsystemBase {
    private final TalonSRX m_pivotMotor;

    private AngularVelocityWrapper m_velocity;

    double m_simVelocity = 0;

    private Supplier<Rotation2d> m_pivotAngle;

    public WristSubsystem(Supplier<Rotation2d> pivotAngle) {
        super("Wrist", WristConstants.TRAPEZOIDAL_CONSTRAINTS, WristConstants.SIMULATION_SUBSYSTEM);
        m_pivotMotor = new TalonSRX(WristConstants.kMotorID);
        m_pivotMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 200);
        m_pivotMotor.enableCurrentLimit(true);
        m_pivotAngle = pivotAngle;
        m_pivotMotor.configPeakCurrentLimit(WristConstants.kMaxCurrent, 500);
        m_pivotMotor.setInverted(false);
        m_velocity = new AngularVelocityWrapper(getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();
        m_velocity.calculate(getAngle());
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
        return Rotation2d
                .fromRotations(m_pivotMotor.getSelectedSensorPosition() / 4096.0 + WristConstants.encoderOffset);
    }

    @Override
    public void setVoltage(double voltage) {
        m_pivotMotor.set(ControlMode.PercentOutput, voltage / 12);
    }

    @Override
    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getAngle().getRadians(), m_velocity.get().getRadians());
    }

    protected void setSimulatedMotors(Matrix<N1, N1> matrix) {
        m_pivotMotor.getSimCollection().setPulseWidthPosition((int) (m_pivotMotor.getSelectedSensorPosition()
                + Units.radiansToRotations(matrix.get(0, 0) * 0.02) * 4096));

    }

    private double getAbsoluteAngleRadians() {
        return getAngle().getRadians() - Math.PI / 2
                + (m_pivotAngle != null ? m_pivotAngle.get().getRadians() - Math.PI : 0);
    }

    @Override
    protected double calculateVoltage(double velocity, double position) {
        double voltage = WristConstants.PID_CONTROLLER.calculate(getAngle().getRadians(), position);
        voltage += WristConstants.FEEDFORWARD_CONTROLLER.calculate(
                Robot.isReal() ? getAbsoluteAngleRadians() : -Math.PI / 2,
                velocity);
        return voltage;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Pivot Angle", () -> {
            return m_pivotAngle != null ? m_pivotAngle.get().getRadians() : 0;
        }, null);

        builder.addDoubleProperty("Absolute Angle", () -> {
            return getAbsoluteAngleRadians();
        }, null);
    }
}
