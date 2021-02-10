package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class ResetOdometry extends InstantCommand {

    private DriveSubsystem m_driveSubsystem;

    private final Pose2d pose2d;

    public ResetOdometry(DriveSubsystem driveSubsystem, Pose2d pose2d) {
        m_driveSubsystem = driveSubsystem;
        this.pose2d = pose2d;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        m_driveSubsystem.resetOdometry(pose2d);
    }
}