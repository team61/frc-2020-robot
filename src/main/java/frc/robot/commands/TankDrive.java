package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TankDrive extends CommandBase {

    private DriveSubsystem m_driveSubsystem;

    private Timer m_timer = new Timer();

    private double m_prevTime;

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
        m_prevTime = 0;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        double leftVolts = m_left.getAsDouble() * AutoConstants.kMaxVoltage;
        double rightVolts = m_right.getAsDouble() * AutoConstants.kMaxVoltage;

        m_driveSubsystem.tankDriveVolts(leftVolts, rightVolts, true);

        double velocity = m_driveSubsystem.getEncoderRate();
        double acceleration = velocity / dt;
        System.out.println(
                "Velocity: " + velocity
                        + "Acceleration: " + acceleration);

        m_prevTime = curTime;
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