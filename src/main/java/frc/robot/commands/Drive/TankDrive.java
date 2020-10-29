package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TankDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;

    private DoubleSupplier m_left;
    private DoubleSupplier m_right;

    public TankDrive(DriveSubsystem driveSubsystem, DoubleSupplier left, DoubleSupplier right) {
        m_driveSubsystem = driveSubsystem;
        m_left = left;
        m_right = right;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_driveSubsystem.tankDrive(m_left.getAsDouble(), m_right.getAsDouble(), true);
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