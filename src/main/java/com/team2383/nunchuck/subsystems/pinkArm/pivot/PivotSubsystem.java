package com.team2383.nunchuck.subsystems.pinkArm.pivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import com.team2383.nunchuck.helpers.TrapezoidalSubsystemBase;
import com.team2383.nunchuck.subsystems.pinkArm.telescope.TelescopeConstants;
import com.team2383.lib.math.Clip;
import com.team2383.nunchuck.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends TrapezoidalSubsystemBase {
    private final PivotIO m_io;
    private final PivotIOInputsAutoLogged m_inputs = new PivotIOInputsAutoLogged();

    private final DoubleSupplier m_extensionSupplier;

    /**
     * Pivot Subsystem Constructor (The pivot is the big swinging joint on the pink
     * arm)
     * 
     * @param extension for safety; checks extension to make sure it's not too long
     */
    public PivotSubsystem(PivotIO io, DoubleSupplier extension) {
        super("Pivot", PivotConstants.TRAPEZOIDAL_CONSTRAINTS, PivotConstants.SIMULATION_SUBSYSTEM,
                PivotConstants.POSITION_THRESHOLD.getRadians());

        m_io = io;

        m_extensionSupplier = extension;

        SmartDashboard.putData("Pivot FF", PivotConstants.FEEDFORWARD_CONTROLLER);
        SmartDashboard.putData("Pivot PID", PivotConstants.PID_CONTROLLER);
    }

    @Override
    public void periodic() {
        super.periodic();
        m_io.updateInputs(m_inputs);

        Logger.getInstance().processInputs("Pivot", m_inputs);
    }

    /**
     * Uses trapezoidal motion-profiling to implement pseudo-positional control
     * Using this method disables velocity control
     * 
     * @param angle
     * @return boolean state to determine whether the input angle is safe
     */
    public boolean setGoal(Rotation2d angle) {
        // safety for upper bounds
        double adjustedAngle = Clip.clip(PivotConstants.LOWER_BOUND.getRadians(), angle.getRadians(),
                PivotConstants.UPPER_BOUND.getRadians());

        // safety for inside robot
        adjustedAngle = (m_extensionSupplier != null ? m_extensionSupplier.getAsDouble()
                : 0) < TelescopeConstants.SAFETY_BOUND ? adjustedAngle
                        : Clip.invClip(PivotConstants.LOWER_SAFETY.getRadians(), adjustedAngle,
                                PivotConstants.UPPER_SAFETY.getRadians());

        super.setGoal(new TrapezoidProfile.State(adjustedAngle, 0));
        return adjustedAngle == angle.getRadians();
    }

    /**
     * Set velocity of the pivot using PID and feedforward control
     * Using this method disables trapezoidal motion profiling
     * 
     * @param desiredVelocity in radians per second
     */
    public void setVelocity(Rotation2d desiredVelocity) {
        super.setVelocity(desiredVelocity.getRadians());
    }

    /**
     * PIDF calculations used by trapezoidal motion profiling
     * 
     * @param velocity in radians per second
     */
    @Override
    protected double calculateVoltage(double velocity, double position) {
        double voltage = PivotConstants.PID_CONTROLLER.calculate(getAngle().getRadians(), position);
        voltage += PivotConstants.FEEDFORWARD_CONTROLLER.calculate(velocity);
        if (Robot.isReal()) { // Am I on a planet with gravity
            voltage += Math.sin(getAngle().getRadians()) * PivotConstants.kG
                    * ((m_extensionSupplier != null ? m_extensionSupplier.getAsDouble() : 0) + PivotConstants.kGOFFSET)
                    / (PivotConstants.kGOFFSET + 20);
        }
        return voltage;
    }

    /**
     * Gets velocity of the abs encoder
     * 
     * @return
     */
    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(m_inputs.velocity);
    }

    /**
     * Returns current angle of offset abs encoder
     * 
     * @return angle in Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_inputs.angle);
    }

    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getAngle().getRadians(), getVelocity().getRadians());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("extension existance", () -> m_extensionSupplier != null, null);
        builder.addDoubleProperty("extension",
                () -> (m_extensionSupplier != null ? m_extensionSupplier.getAsDouble() : 0), null);
    }

    /**
     * Sets the voltage of the pivot motors
     * 
     * @param voltage
     */
    @Override
    protected void setVoltage(double voltage) {
        m_io.setVoltage(voltage);
    }
}
