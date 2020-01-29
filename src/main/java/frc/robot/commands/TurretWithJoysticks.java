package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretWithJoysticks extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private DoubleSupplier m_angle;
    private DoubleSupplier m_heading;

    public TurretWithJoysticks(TurretSubsystem turretSubsystem, DoubleSupplier angle, DoubleSupplier heading) {
        m_turretSubsystem = turretSubsystem;
        m_angle = angle;
        m_heading = heading;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        m_turretSubsystem.set(m_heading.getAsDouble(), m_angle.getAsDouble());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turretSubsystem.stop();
    }
}