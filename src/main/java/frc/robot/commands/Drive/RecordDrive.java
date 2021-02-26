package frc.robot.commands.Drive;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.AutoConstants;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;
import java.util.Scanner;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RecordDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private SendableChooser<Command> m_chooser;


    private DoubleSupplier m_left;
    private DoubleSupplier m_right;
    private ArrayList<double[]> speeds = new ArrayList<double[]>();
    public RecordDrive(DriveSubsystem driveSubsystem, DoubleSupplier left, DoubleSupplier right, SendableChooser<Command> chooser) {
        m_driveSubsystem = driveSubsystem;
        m_chooser = chooser;
        m_left = left;
        m_right = right;
    
        addRequirements(m_driveSubsystem);
    }
   

   
    //"/home/lvuser/";
  private String fileName;
    @Override
    public void initialize() {
        super.initialize();
      fileName = m_driveSubsystem.getFileName();
    
        try {
           File file = new File(AutoConstants.directory + fileName);
          
            if (file.createNewFile()) {
                System.out.println("File created: " + file.getName());
              } else {
                file.delete();
                file = new File(AutoConstants.directory + fileName);
                System.out.println("File overridden: " + file.getName());
              }  
    } catch (IOException e) {
        System.out.println("An error occurred.");
        e.printStackTrace();
    }
    m_driveSubsystem.setRecording(true);
    }

    @Override
    public void execute() {
    
    
            
              m_driveSubsystem.tankDriveVolt(m_left.getAsDouble(), m_right.getAsDouble());
           


              speeds.add(new double[]{m_left.getAsDouble(), m_right.getAsDouble()});
            //System.out.print(m_driveSubsystem.getLeftMotorOutput() + "," + m_driveSubsystem.getRightMotorOutput() + " ");
           // System.out.print("new double[]{" + m_left.getAsDouble() +"," + m_right.getAsDouble() + "},");
           
         

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_driveSubsystem.getRecording();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       
        try {
            
                File file = new File(AutoConstants.directory + fileName);
             
               FileWriter writer = new FileWriter(file, true);
               for(double[] speed : speeds) {
                 
                writer.write(speed[0] + "," + speed[1] + "\n");
               }
                
                writer.close();
                System.out.println("Successfully wrote to the file.");
           
    
            
            File listFile = new File(AutoConstants.directory + "listFile.txt");
           
            FileWriter listFileWriter = new FileWriter(listFile, true);
            Scanner reader = new Scanner(listFile);
            boolean containsFile = false;
            while (reader.hasNextLine()) {
              String data = reader.nextLine();
              if (data.contentEquals(fileName)) {
containsFile = true;
break;
              }
            }
            reader.close();
           if (!containsFile) {
            listFileWriter.write(fileName + "\n");
           }
             listFileWriter.close();
             
            System.out.println("Successfully ended.");
          } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }
        // String message = "";
        // System.out.println("Start");
        // for(double[] coord : coords) {
        //     message = message + "new double[]{" + coord[0] +"," + coord[1] + "},";
        // }
        // System.out.println("Start");
        // System.out.print(message);
        // System.out.println("End");

        // System.out.println("Start");
        // System.out.print(message);
        // System.out.println("End");
        m_chooser.addOption(fileName, new ReadDrive(m_driveSubsystem, fileName));
        m_driveSubsystem.stopTankDrive();
    }


}

