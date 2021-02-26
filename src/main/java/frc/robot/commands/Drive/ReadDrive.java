package frc.robot.commands.Drive;

import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;

public class ReadDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private int count;
    private ArrayList<double[]> speeds = new ArrayList<double[]>();
    final String m_fileName;
    public ReadDrive(DriveSubsystem driveSubsystem, String fileName) {
        m_driveSubsystem = driveSubsystem;
        m_fileName = fileName;

        addRequirements(m_driveSubsystem);
    }


    @Override
    public void initialize() {
        count = 0;
        try {
            File file = new File(AutoConstants.directory + m_fileName);
           
             if (file.createNewFile()) {
                 System.out.println("File doesn't exsist: " + file.getName());
                 end(true);
               } 
               Scanner reader = new Scanner(file);
        
            while (reader.hasNextLine()) {
              String data = reader.nextLine();
              String[] values = data.split(",");
            //   System.out.println(data);
            //   System.out.println(values[0] +  "," + values[1]);
              speeds.add(new double[]{Double.parseDouble(values[0]), Double.parseDouble(values[1])});
            
            }
            reader.close();
     } catch (IOException e) {
         System.out.println("An error occurred.");
         e.printStackTrace();
     }
    }

    @Override
    public void execute() {
            m_driveSubsystem.tankDriveVolt(speeds.get(count)[0], speeds.get(count)[1]);
    
            count++;
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return count == speeds.size() - 1;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopTankDrive();
    }
}