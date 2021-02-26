package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopRecording extends InstantCommand {
    private DriveSubsystem m_driveSubsystem;

    public StopRecording(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
    
    }

    @Override
    public void execute() {
        m_driveSubsystem.setRecording(false);
    }
}