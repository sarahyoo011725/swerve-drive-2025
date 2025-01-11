package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.config;

public class trajectories {

   public static final Trajectory test_traj() {
    return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d()),
            List.of(
                new Translation2d(1, 0),
                new Translation2d(2, 1)
            ),
            new Pose2d(2, 1, Rotation2d.fromDegrees(45)),
            config.swerve.trajectory_config()
        );
   }
}
