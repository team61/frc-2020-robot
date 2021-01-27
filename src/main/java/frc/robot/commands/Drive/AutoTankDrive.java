package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.function.DoubleSupplier;

public class AutoTankDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;

    private DoubleSupplier m_left;
    private DoubleSupplier m_right;

    public AutoTankDrive(DriveSubsystem driveSubsystem, DoubleSupplier left, DoubleSupplier right) {
        m_driveSubsystem = driveSubsystem;
        m_left = left;
        m_right = right;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_driveSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    }

    @Override
    public void execute() {

        m_driveSubsystem.tankDriveVolts(m_left.getAsDouble(), m_right.getAsDouble());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopTankDrive();
    }
}