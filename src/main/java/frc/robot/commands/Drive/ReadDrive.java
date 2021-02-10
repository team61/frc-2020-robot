package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;

public class ReadDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;

    private String fileName;
    private int count = 0;
    private ArrayList<double[]> speeds = new ArrayList<double[]>();
    ReadDrive(DriveSubsystem driveSubsystem, String fileName) {
        m_driveSubsystem = driveSubsystem;
        this.fileName = fileName;
        addRequirements(m_driveSubsystem);
    }



    @Override
    public void initialize() {
        try {
            File file = new File(fileName);
            Scanner myReader = new Scanner(file);
            while (myReader.hasNextLine()) {
                String data = myReader.nextLine();
                String[] values = data.split(",");
                double left = Double.parseDouble(values[0]);
                double right = Double.parseDouble(values[1]);
                speeds.add(new double[]{left, right});
                System.out.println(data);
            }
            myReader.close();
        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
            m_driveSubsystem.tankDrive(speeds.get(count)[0], speeds.get(count)[1]);
            count++;
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return count < speeds.size();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopTankDrive();
    }
}