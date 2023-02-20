package com.team2383.diffy.subsystems.pinkArm.wrist;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2383.diffy.helpers.AngularVelocityWrapper;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.TrapezoidalSubsystemBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
public class WristSubsystem extends TrapezoidalSubsystemBase {
    private final TalonSRX m_pivotMotor;

    private double m_voltage;

    private AngularVelocityWrapper m_velocity;

    double m_simVelocity = 0;

    private final Supplier<Rotation2d> m_pivotAngle;

    public WristSubsystem(Supplier<Rotation2d> pivotAngle) {
        super("Wrist", WristConstants.TRAPEZOIDAL_CONSTRAINTS, WristConstants.SIMULATION_SUBSYSTEM);
        m_pivotMotor = new TalonSRX(WristConstants.kMotorID);
        m_pivotMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 200);
        m_pivotMotor.enableCurrentLimit(true);
        m_pivotAngle = pivotAngle;
        m_pivotMotor.configPeakCurrentLimit(20, 500);
        m_velocity = new AngularVelocityWrapper(getAngle());
    }

    public void periodic() {
        m_velocity.calculate(getAngle());
    }

    public void setGoal(Rotation2d desiredAngle) {
        // safety for upper bounds
        double adjustedAngle = Clip.clip(WristConstants.kLowerBound.getRadians(), desiredAngle.getRadians(), WristConstants.kUpperBound.getRadians());
        super.setGoal(new TrapezoidProfile.State(adjustedAngle, 0));
    }

    public void setVelocity(double velocity) {
        super.setVelocity(velocity);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations((m_pivotMotor.getSelectedSensorPosition() + WristConstants.encoderOffset) / 4000.0);
    }

    public void setVoltage() {
        m_pivotMotor.set(ControlMode.PercentOutput, m_voltage / 12);
    }

    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getAngle().getRadians(), m_velocity.get().getRadians());
    }

    protected void setSimulatedMotors(Matrix<N1, N1> matrix) {
        //TODO: Actually do sim support
        m_pivotMotor.set(TalonSRXControlMode.Velocity, matrix.get(0, 0));
        
    }

    protected double calculateVoltage(double velocity) {
        double voltage = WristConstants.PID_CONTROLLER.calculate(m_velocity.get().getRadians(), velocity);
        voltage += WristConstants.FEEDFORWARD_CONTROLLER.calculate(getAngle().getRadians() + m_pivotAngle.get().getRadians() + 90, velocity);
        return voltage;
    }

    protected void setVoltage(double voltage) {
        m_pivotMotor.set(ControlMode.PercentOutput, voltage / 12);
    }
}
