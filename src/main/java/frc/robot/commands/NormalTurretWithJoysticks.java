package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class NormalTurretWithJoysticks extends CommandBase {

    private Turret m_turret;

    private DoubleSupplier m_x;

    public NormalTurretWithJoysticks(Turret turret, DoubleSupplier x) {
        m_turret = turret;
        m_x = x;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        m_turret.set(m_x.getAsDouble());
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