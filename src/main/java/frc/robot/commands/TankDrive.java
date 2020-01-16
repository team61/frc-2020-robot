package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class TankDrive extends CommandBase {

    private DriveTrain m_driveTrain;

    private DoubleSupplier m_left;
    private DoubleSupplier m_right;

    public TankDrive(DriveTrain driveTrain, DoubleSupplier left, DoubleSupplier right) {
        m_driveTrain = driveTrain;
        m_left = left;
        m_right = right;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        m_driveTrain.tankDriveSquared(m_left.getAsDouble(), m_right.getAsDouble());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stopTankDrive();
    }
}