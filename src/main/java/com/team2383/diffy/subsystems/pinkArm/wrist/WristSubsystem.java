package com.team2383.diffy.subsystems.pinkArm.wrist;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.TrapezoidalSubsystemBase;
// import com.revrobotics.PowerDistribution.voltage;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;

public class WristSubsystem extends TrapezoidalSubsystemBase {
    private final TalonSRX m_pivotMotor;

    double m_simVelocity = 0;

    private Supplier<Rotation2d> m_pivotAngle;
    private Rotation2d m_startRotation = new Rotation2d();

    public WristSubsystem(Supplier<Rotation2d> pivotAngle) {
        super("Wrist", WristConstants.TRAPEZOIDAL_CONSTRAINTS, WristConstants.SIMULATION_SUBSYSTEM);
        m_pivotMotor = new TalonSRX(WristConstants.kMotorID);
        m_pivotMotor.configFactoryDefault();
        // Set postional offset

        m_pivotMotor.configPeakCurrentLimit(WristConstants.kMaxCurrent, 500);
        m_pivotMotor.enableCurrentLimit(true);
        m_pivotMotor.configVoltageCompSaturation(12);
        m_pivotMotor.enableVoltageCompensation(true);
        m_pivotMotor.configVoltageMeasurementFilter(1);
        m_pivotAngle = pivotAngle;

        m_pivotMotor.setInverted(false);
        m_startRotation = Rotation2d.fromRotations(
                (m_pivotMotor.getSensorCollection().getPulseWidthPosition()
                        - m_pivotMotor.getSensorCollection().getQuadraturePosition()) / 4096.0
                        + WristConstants.encoderOffset.getRotations());
    }

    @Override
    public void periodic() {
        super.periodic();
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
                .fromRotations(m_pivotMotor.getSensorCollection().getQuadraturePosition() / 4096.0
                        + m_startRotation.getRotations());
    }

    @Override
    public void setVoltage(double voltage) {
        m_pivotMotor.set(ControlMode.PercentOutput, voltage / 12);
    }

    @Override
    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getAngle().getRadians(), getVelocity().getRadians());
    }

    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(m_pivotMotor.getSensorCollection().getQuadratureVelocity() / 4096.0 * 10);
    }

    protected void setSimulatedMotors(Matrix<N1, N1> matrix) {
        m_pivotMotor.getSimCollection().setPulseWidthPosition((int) (m_pivotMotor.getSelectedSensorPosition()
                + Units.radiansToRotations(matrix.get(0, 0) * 0.02) * 4096));

    }

    private double getAbsoluteAngleRadians() {
        return getAngle().getRadians() - Math.PI / 2
                + (m_pivotAngle != null ? m_pivotAngle.get().getRadians() + Math.PI : 0);
    }

    @Override
    protected double calculateVoltage(double velocity, double position) {
        double voltage = -WristConstants.PID_CONTROLLER.calculate(getAngle().getRadians(), position);
        voltage -= WristConstants.FEEDFORWARD_CONTROLLER.calculate(
                getAbsoluteAngleRadians(),
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
