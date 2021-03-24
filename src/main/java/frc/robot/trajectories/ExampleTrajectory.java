package frc.robot.trajectories;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;


public class ExampleTrajectory {
    public static Trajectory generateTrajectory() {
        
      // 2018 cross scale auto waypoints.
      Pose2d start = new Pose2d(0, 0,
          Rotation2d.fromDegrees(0));
          Pose2d end = new Pose2d(0, 1,
          Rotation2d.fromDegrees(0));
  
      var interiorWaypoints = new ArrayList<Translation2d>();
       interiorWaypoints.add(new Translation2d(2, 4));
       interiorWaypoints.add(new Translation2d(1, 2));

        DifferentialDriveVoltageConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(AutoConstants.kS,
                                AutoConstants.kV,
                                AutoConstants.kA),
                        AutoConstants.kDriveKinematics,
                        AutoConstants.kMaxVoltage);

        TrajectoryConfig config =
                new TrajectoryConfig(AutoConstants.kMaxVelocity,
                        AutoConstants.kMaxAcceleration).setKinematics(AutoConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);
                        // Add kinematics to ensure max speed is actually obeyed
                        // .setKinematics(AutoConstants.kDriveKinematics)
                        // // Apply the voltage constraint
                        // .addConstraint(autoVoltageConstraint);
     
  

      return TrajectoryGenerator.generateTrajectory(
          start,
          interiorWaypoints,
          end,
          config);
    }
  }