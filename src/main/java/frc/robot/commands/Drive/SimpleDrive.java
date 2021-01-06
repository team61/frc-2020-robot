package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class SimpleDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;

    private double m_speed;
    private double m_distance;

    public SimpleDrive(DriveSubsystem driveSubsystem, double speed, double distance) {
        m_driveSubsystem = driveSubsystem;
        m_speed = speed;
        m_distance = distance;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {

        m_driveSubsystem.tankDrive(0.2, .2);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_driveSubsystem.getDistanceTraveled()) > Math.abs(m_distance);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_driveSubsystem.stopTankDrive();
        }
    }
}