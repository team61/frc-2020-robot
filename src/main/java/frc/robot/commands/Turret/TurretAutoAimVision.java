package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretAutoAimVision extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private DoubleSupplier m_yaw;

    public TurretAutoAimVision(TurretSubsystem turretSubsystem, DoubleSupplier yaw) {
        m_turretSubsystem = turretSubsystem;
        m_yaw = yaw;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        if (m_yaw.getAsDouble() > TurretConstants.kVisionTolerance) {
            m_turretSubsystem.setVoltage(TurretConstants.kVisionVoltage);
        } else if (m_yaw.getAsDouble() < -TurretConstants.kVisionTolerance) {
            m_turretSubsystem.setVoltage(-TurretConstants.kVisionVoltage);
        } else {
            m_turretSubsystem.stop();
        }
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
