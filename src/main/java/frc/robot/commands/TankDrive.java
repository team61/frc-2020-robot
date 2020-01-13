package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

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
}