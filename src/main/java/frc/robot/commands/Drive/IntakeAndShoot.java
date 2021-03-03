package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Feed.Feed;
import frc.robot.commands.Feed.Intake;
import frc.robot.commands.Shoot.Shoot;
import frc.robot.commands.Turret.TurretAutoAimVision;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.function.DoubleSupplier;

public class IntakeAndShoot extends SequentialCommandGroup {

    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final TurretSubsystem m_turretSubsystem = TurretSubsystem.getInstance();
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final VisionSubsystem m_visionSubsystem = VisionSubsystem.getInstance();
    public IntakeAndShoot() {
        addCommands(
        new Intake(m_feederSubsystem), 
        new ParallelDeadlineGroup(new SimpleDrive(m_driveSubsystem, -4, -2),new Shoot(m_shooterSubsystem, ShooterConstants.autoVoltages[2]),new TurretAutoAimVision(m_turretSubsystem, m_visionSubsystem::getYaw)),
        new ParallelDeadlineGroup(new ParallelDeadlineGroup(new WaitCommand(2.5), new Feed(m_feederSubsystem)),
                        new Shoot(m_shooterSubsystem, ShooterConstants.autoVoltages[2]),
                        new TurretAutoAimVision(m_turretSubsystem, m_visionSubsystem::getYaw)),
                new SimpleDrive(m_driveSubsystem, 4, 2));
    }

}