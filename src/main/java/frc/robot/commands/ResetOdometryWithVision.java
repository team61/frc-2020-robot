package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import lib.util.Pose2dConsumer;

import java.util.function.DoubleSupplier;

public class ResetOdometryWithVision extends InstantCommand {

    private double m_distance;
    private Pose2d m_curPosition;
    private Pose2dConsumer m_pose2dConsumer;

    public ResetOdometryWithVision(double distance, Pose2d curPosition, Pose2dConsumer pose2dConsumer) {
        m_distance = distance;
        m_curPosition = curPosition;
        m_pose2dConsumer = pose2dConsumer;
    }

    @Override
    public void execute() {
        Translation2d curTranslation2d = m_curPosition.getTranslation();
        double heading = m_curPosition.getRotation().getRadians();

        double newX = curTranslation2d.getX() + Math.cos(heading) * m_distance;
        double newY = curTranslation2d.getY() + Math.sin(heading) * m_distance;

        Pose2d newPose2d = new Pose2d(newX, newY, new Rotation2d(heading));
        m_pose2dConsumer.accept(newPose2d);
    }
}
