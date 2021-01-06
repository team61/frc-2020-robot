package frc.robot.trajectories;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.AutoConstants;


public class ExampleTrajectory {
    public static Trajectory generateTrajectory() {
        
      // 2018 cross scale auto waypoints.
      Pose2d sideStart = new Pose2d(0, 0,
          Rotation2d.fromDegrees(0));
          Pose2d crossScale = new Pose2d(Units.feetToMeters(6), Units.feetToMeters(6),
          Rotation2d.fromDegrees(0));
  
      var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(2, 2));
      interiorWaypoints.add(new Translation2d(4, 3));
  
      TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration);
     
  

      return TrajectoryGenerator.generateTrajectory(
          sideStart,
          interiorWaypoints,
          crossScale,
          config);
    }
  }