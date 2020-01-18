package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretWithJoysticks extends CommandBase {

    private Turret m_turret;

    private DoubleSupplier m_angle;
    private DoubleSupplier m_heading;

    public TurretWithJoysticks(Turret turret, DoubleSupplier angle, DoubleSupplier heading) {
        m_turret = turret;
        m_angle = angle;
        m_heading = heading;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        m_turret.set(m_heading.getAsDouble(), m_angle.getAsDouble());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }
}