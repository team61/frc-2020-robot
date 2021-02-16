package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;

public class ReadDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private int count;
    private double[][] m_speeds;

    public ReadDrive(DriveSubsystem driveSubsystem, double[][] speeds) {
        m_driveSubsystem = driveSubsystem;
        m_speeds = speeds;

        addRequirements(m_driveSubsystem);
    }


    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
            m_driveSubsystem.tankDriveVolt(m_speeds[count][0], m_speeds[count][1]);
    
            count++;
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return count == m_speeds.length - 1;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopTankDrive();
    }
}