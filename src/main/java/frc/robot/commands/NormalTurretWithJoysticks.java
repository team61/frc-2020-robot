package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

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

        m_turret.setSpeed(m_x.getAsDouble());
    }
}