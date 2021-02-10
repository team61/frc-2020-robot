package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RecordDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    final String fileName;

    private DoubleSupplier m_left;
    private DoubleSupplier m_right;
    private BooleanSupplier m_finished;

    public RecordDrive(DriveSubsystem driveSubsystem, String fileName, DoubleSupplier left, DoubleSupplier right, BooleanSupplier finished) {
        m_driveSubsystem = driveSubsystem;
        this.fileName = fileName;
        m_left = left;
        m_right = right;
        m_finished = finished;
        addRequirements(m_driveSubsystem);
    }


    @Override
    public void initialize() {
        m_driveSubsystem.tankDrive(m_left.getAsDouble(), m_right.getAsDouble(), true);
        try {
            File file = new File(fileName);
            if (file.createNewFile()) {
                System.out.println("File created: " + file.getName());
            } else {
                System.out.println("File already exists.");
            }
            //fw = new FileWriter(fileName, true);
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

    }
    FileWriter fw;

    @Override
    public void execute() {
            // System.out.println("recording");
            // try {
               
            //     fw.write(m_driveSubsystem.getLeftMotorOutput() + "," + m_driveSubsystem.getRightMotorOutput() + "\n");
            
            //     System.out.println("Successfully wrote to the file.");
            // } catch (IOException e) {
            //     System.out.println("An error occurred.");
            //     e.printStackTrace();
            // }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_finished.getAsBoolean();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // try {
        //     fw.close();
        // } catch(IOException e) {
        //     System.out.println(e);
        // }
        m_driveSubsystem.stopTankDrive();
    }


}

