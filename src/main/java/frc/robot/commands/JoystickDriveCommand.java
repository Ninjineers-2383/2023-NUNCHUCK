package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.ThrottleSoftener;
import frc.robot.subsystems.DrivetrainSubsystem;

public class JoystickDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrain;

    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    private final DoubleSupplier m_omega;
    private final BooleanSupplier m_fieldRelative;

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(100);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(100);
    private final SlewRateLimiter m_oRateLimiter = new SlewRateLimiter(100);

    public JoystickDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier xInput, DoubleSupplier yInput,
            DoubleSupplier rotationInput, BooleanSupplier fieldRelative) {
        m_drivetrain = drivetrain;

        m_x = xInput;
        m_y = yInput;
        m_omega = rotationInput;
        m_fieldRelative = fieldRelative;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double x = -ThrottleSoftener.soften(MathUtil.applyDeadband(m_x.getAsDouble(), 0.1)) * DriveConstants.kMaxSpeed;
        double y = -ThrottleSoftener.soften(MathUtil.applyDeadband(m_y.getAsDouble(), 0.1)) * DriveConstants.kMaxSpeed;
        double omega = -ThrottleSoftener.soften(MathUtil.applyDeadband(m_omega.getAsDouble(), 0.1))
                * DriveConstants.kMaxAngularSpeed;

        m_drivetrain.drive(
                m_xRateLimiter.calculate(-y),
                m_yRateLimiter.calculate(x),
                m_oRateLimiter.calculate(omega),
                m_fieldRelative.getAsBoolean());
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
