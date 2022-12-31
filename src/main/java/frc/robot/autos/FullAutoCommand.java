package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FullAutoCommand extends SequentialCommandGroup {
    public FullAutoCommand(DrivetrainSubsystem drivetrain, String pathName, HashMap<String, Command> commandMap) {

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(4, 3));
        Pose2d initialPos = pathGroup.get(0).getInitialHolonomicPose();
        drivetrain.resetOdometry(initialPos);

        for (PathPlannerTrajectory trajectory : pathGroup) {
            addCommands(new TrajectoryDriveCommand(drivetrain, trajectory, commandMap));
        }
    }
}
