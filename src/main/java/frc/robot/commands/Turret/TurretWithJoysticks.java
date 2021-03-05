package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretWithJoysticks extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private DoubleSupplier m_speed;
    public TurretWithJoysticks(TurretSubsystem turretSubsystem, DoubleSupplier speed) {
        m_turretSubsystem = turretSubsystem;
        m_speed = speed;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {

        double targetSpeed =m_turretSubsystem.getMaxSpeed() * m_speed.getAsDouble();
        //m_turretSubsystem.set(Math.copySign(m_speed.getAsDouble() * m_speed.getAsDouble(), m_speed.getAsDouble()) * 0.5);
   m_turretSubsystem.setVoltage(m_turretSubsystem.getOutput(targetSpeed));
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