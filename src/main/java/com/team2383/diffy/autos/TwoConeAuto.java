package com.team2383.diffy.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.team2383.diffy.subsystems.DrivetrainSubsystem;

public class TwoConeAuto extends SequentialCommandGroup {
    public TwoConeAuto(DrivetrainSubsystem drivetrain, String pathName, SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName,
                new PathConstraints(1, 0.5));

        addCommands(autoBuilder.fullAuto(pathGroup));
    }
}
