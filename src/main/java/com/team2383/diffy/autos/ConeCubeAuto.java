package com.team2383.diffy.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.diffy.commands.FeederCommand;
import com.team2383.diffy.commands.pinkArm.PinkArmPresetCommand;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants;
import com.team2383.diffy.commands.pinkArm.zero.ZeroTelescope;
import com.team2383.diffy.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.diffy.subsystems.pinkArm.feeder.FeederSubsystem;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeCubeAuto extends SequentialCommandGroup {
    public ConeCubeAuto(DrivetrainSubsystem m_drivetrain, TelescopeSubsystem telescope, PivotSubsystem pivot,
            WristSubsystem wrist, FeederSubsystem feeder, SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ConeCube",
                new PathConstraints(1, 1));
        addCommands(
                new InstantCommand(() -> m_drivetrain.forceHeading(Rotation2d.fromDegrees(10.5))),
                new ZeroTelescope(telescope),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.MID_SCORE_BACK),
                new PinkArmPresetCommand(pivot, telescope, wrist, PositionConstants.HIGH_SCORE_BACK),
                new FeederCommand(feeder, () -> -1).withTimeout(0.4),
                new FeederCommand(feeder, () -> 0).withTimeout(0.01),
                autoBuilder.fullAuto(pathGroup),
                new InstantCommand(() -> m_drivetrain.resetHeading()));
    }
}
