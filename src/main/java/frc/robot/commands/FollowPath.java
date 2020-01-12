package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class FollowPath extends CommandBase {

    private DriveTrain m_driveTrain;

    private String m_pathName;

    public FollowPath(DriveTrain driveTrain, String pathName) {
        m_driveTrain = driveTrain;
        m_pathName = pathName;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
      try {
        Trajectory left_trajectory = PathfinderFRC.getTrajectory(m_pathName + ".left");
        Trajectory right_trajectory = PathfinderFRC.getTrajectory(m_pathName + ".right");
    
        m_driveTrain.m_left_follower = new EncoderFollower(left_trajectory);
        m_driveTrain.m_right_follower = new EncoderFollower(right_trajectory);
    
        m_driveTrain.m_left_follower.configureEncoder(m_driveTrain.getLeftEncoder(), DriveConstants.kEncoderCPR, DriveConstants.kWheelDiameter);
        
        m_driveTrain.m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / DriveConstants.kMaxVelocity, 0);
    
        m_driveTrain.m_right_follower.configureEncoder(m_driveTrain.getRightEncoder(), DriveConstants.kEncoderCPR, DriveConstants.kWheelDiameter);

        m_driveTrain.m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / DriveConstants.kMaxVelocity, 0);
    
        m_driveTrain.m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    @Override
    public void execute() {
          double left_speed = m_driveTrain.m_left_follower.calculate(m_driveTrain.getLeftEncoder());
          double right_speed = m_driveTrain.m_right_follower.calculate(m_driveTrain.getRightEncoder());
          double heading = m_driveTrain.getAngle();
          double desired_heading = Pathfinder.r2d(m_driveTrain.m_left_follower.getHeading());
          double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
          double turn =  0.8 * (-1.0/80.0) * heading_difference;
          m_driveTrain.setLeftSpeed(left_speed + turn);
          m_driveTrain.setRightSpeed(right_speed - turn);
    }

    @Override
    public boolean isFinished() {
      return m_driveTrain.m_left_follower.isFinished() || m_driveTrain.m_right_follower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
      m_driveTrain.m_follower_notifier.stop();
    }
}