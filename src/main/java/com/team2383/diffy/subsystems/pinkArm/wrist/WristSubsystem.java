package com.team2383.diffy.subsystems.pinkArm.wrist;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
    private final TalonFX m_wristMotor;

    double m_simVelocity = 0;

    private Supplier<Rotation2d> m_pivotAngle;

    public WristSubsystem(Supplier<Rotation2d> pivotAngle) {
        super("Wrist", WristConstants.TRAPEZOIDAL_CONSTRAINTS, WristConstants.SIMULATION_SUBSYSTEM,
                WristConstants.POSITON_THRESHOLD.getRadians());
        m_wristMotor = new TalonFX(WristConstants.kMotorID);
        m_wristMotor.configFactoryDefault();
        // Set postional offset

        m_wristMotor.configSupplyCurrentLimit(WristConstants.kCurrentLimit, 500);
        m_wristMotor.configVoltageCompSaturation(12);
        m_wristMotor.enableVoltageCompensation(true);
        m_wristMotor.configVoltageMeasurementFilter(1);
        m_pivotAngle = pivotAngle;
        // m_wristMotor.getSensorCollection().setQuadraturePosition(m_wristMotor.getSensorCollection().getPulseWidthPosition(),
        // 200);
        // Resets position of integrated sensor when robot is turned on
        m_wristMotor.getSensorCollection().setIntegratedSensorPosition(0, 200);

        m_wristMotor.setInverted(false);
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
                .fromRotations((m_wristMotor.getSensorCollection().getIntegratedSensorPosition()
                        - WristConstants.encoderOffset) / 2048.0);
    }

    @Override
    public void setVoltage(double voltage) {
        m_wristMotor.set(ControlMode.PercentOutput, voltage / 12);
    }

    @Override
    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getAngle().getRadians(), getVelocity().getRadians());
    }

    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(m_wristMotor.getSensorCollection().getIntegratedSensorVelocity() / 2048.0 * 10);
    }

    protected void setSimulatedMotors(Matrix<N1, N1> matrix) {
        m_wristMotor.getSimCollection().setIntegratedSensorRawPosition((int) (m_wristMotor.getSelectedSensorPosition()
                + Units.radiansToRotations(matrix.get(0, 0) * 0.02) * 2048));

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

        builder.addDoubleProperty("ABS Raw", m_wristMotor.getSensorCollection()::getIntegratedSensorPosition, null);

        builder.addDoubleProperty("Quad Raw", m_wristMotor.getSensorCollection()::getIntegratedSensorPosition, null);

    }
}
