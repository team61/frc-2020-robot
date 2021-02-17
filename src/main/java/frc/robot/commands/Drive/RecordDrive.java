package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RecordDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;


    private DoubleSupplier m_left;
    private DoubleSupplier m_right;
    private BooleanSupplier m_finished;
    private ArrayList<double[]> coords = new ArrayList<double[]>();
    private ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public RecordDrive(DriveSubsystem driveSubsystem, DoubleSupplier left, DoubleSupplier right, BooleanSupplier finished) {
        m_driveSubsystem = driveSubsystem;
    
        m_left = left;
        m_right = right;
        m_finished = finished;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.println("Start");
    }

    @Override
    public void execute() {
            // System.out.println("recording");
            // try {
              m_driveSubsystem.tankDriveVolt(m_left.getAsDouble(), m_right.getAsDouble());
           
               coords.add(new double[]{m_left.getAsDouble(), m_right.getAsDouble()});
            //System.out.print(m_driveSubsystem.getLeftMotorOutput() + "," + m_driveSubsystem.getRightMotorOutput() + " ");
            System.out.print("new double[]{" + m_left.getAsDouble() +"," + m_right.getAsDouble() + "},");
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
        System.out.println("End");
        String message = "";
        System.out.println("Start");
        for(double[] coord : coords) {
            message = message + "new double[]{" + coord[0] +"," + coord[1] + "},";
        }
        System.out.println("Start");
        System.out.print(message);
        System.out.println("End");

        System.out.println("Start");
        System.out.print(message);
        System.out.println("End");

        // System.out.println("Start");
        // System.out.print(message);
        // System.out.println("End");
        m_driveSubsystem.stopTankDrive();
    }


}

