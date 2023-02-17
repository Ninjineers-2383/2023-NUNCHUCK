package com.team2383.diffy.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2383.diffy.Constants;
import com.team2383.diffy.helpers.ThrottleSoftener;
import com.team2383.diffy.subsystems.Drivetrain.DriveConstants;
import com.team2383.diffy.subsystems.Drivetrain.DrivetrainSubsystem;

public class JoystickDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrain;

    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    private final DoubleSupplier m_omega;
    private final BooleanSupplier m_fieldRelative;
    private final IntSupplier m_hatSupplier;

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(50);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(50);
    private final SlewRateLimiter m_oRateLimiter = new SlewRateLimiter(100);

    public JoystickDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier xInput, DoubleSupplier yInput,
            DoubleSupplier rotationInput, BooleanSupplier fieldRelative, IntSupplier hatSupplier) {
        m_drivetrain = drivetrain;

        m_x = xInput;
        m_y = yInput;
        m_omega = rotationInput;
        m_fieldRelative = fieldRelative;
        m_hatSupplier = hatSupplier;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double x = -ThrottleSoftener.soften(m_x.getAsDouble())
                * DriveConstants.kMaxVelocity;
        double y = -ThrottleSoftener.soften(m_y.getAsDouble())
                * DriveConstants.kMaxVelocity;
        double omega = -ThrottleSoftener.soften(m_omega.getAsDouble())
                * DriveConstants.kMaxAngularVelocity;

        int hatPosition = m_hatSupplier.getAsInt();

        m_drivetrain.drive(
                m_xRateLimiter.calculate(y),
                m_yRateLimiter.calculate(x),
                m_oRateLimiter.calculate(omega),
                m_fieldRelative.getAsBoolean(),
                getCenterOfRotation(hatPosition));
    }

    private Translation2d getCenterOfRotation(int hatPosition) {
        if (!(hatPosition >= 0 && hatPosition <= 360)) {
            return new Translation2d(0, 0);
        }
        // Allows rotating around the swerve modules
        Translation2d pos = new Translation2d((Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 3, 0);
        pos = pos.rotateBy(Rotation2d.fromDegrees(-hatPosition));
        return new Translation2d((Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 12, 0).plus(pos);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.motorsOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
