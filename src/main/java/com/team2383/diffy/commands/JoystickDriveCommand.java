package com.team2383.diffy.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2383.diffy.helpers.ThrottleSoftener;
import com.team2383.diffy.subsystems.drivetrain.DriveConstants;
import com.team2383.diffy.subsystems.drivetrain.DrivetrainSubsystem;

public class JoystickDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrain;

    private final Supplier<Rotation2d> m_rotSupply;
    private final Supplier<Translation2d> m_moveSupply;
    private final BooleanSupplier m_fieldRelative;
    private final IntSupplier m_hatSupplier;
    private Rotation2d m_integralStuff = Rotation2d.fromDegrees(0);

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_oRateLimiter = new SlewRateLimiter(10);
    private final double lastTime;

    public JoystickDriveCommand(DrivetrainSubsystem drivetrain, Supplier<Translation2d> moveSupplier,
            Supplier<Rotation2d> rotation, BooleanSupplier fieldRelative, IntSupplier hatSupplier) {
        m_drivetrain = drivetrain;

        m_moveSupply = moveSupplier;
        m_rotSupply = rotation;
        m_fieldRelative = fieldRelative;
        m_hatSupplier = hatSupplier;

        addRequirements(m_drivetrain);

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        Translation2d move = m_moveSupply.get();
        double x = -ThrottleSoftener.soften(move.getX())
                * DriveConstants.kMaxVelocity;
        double y = -ThrottleSoftener.soften(move.getY())
                * DriveConstants.kMaxVelocity;
        double deltaTime = Timer.getFPGATimestamp() - lastTime;
        m_integralStuff = m_integralStuff.plus(
                Rotation2d.fromRadians(-ThrottleSoftener.soften(m_rotSupply.get().getRadians())).times(1 / deltaTime));

        int hatPosition = m_hatSupplier.getAsInt();

        Rotation2d rotVelocity = Rotation2d.fromRadians(
                m_oRateLimiter.calculate(
                        DriveConstants.HEADING_CONTROLLER.calculate(m_drivetrain.getHeading().getRadians(),
                                m_integralStuff.getRadians())));

        m_drivetrain.drive(
                new Translation2d(
                        m_xRateLimiter.calculate(x),
                        m_yRateLimiter.calculate(y)),
                rotVelocity,
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
