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
    final private boolean m_endOnFinish;

    public TurretAutoAimVision(TurretSubsystem turretSubsystem, DoubleSupplier yaw, boolean endOnFinish) {
        m_turretSubsystem = turretSubsystem;
        m_yaw = yaw;
        m_endOnFinish = endOnFinish;

        addRequirements(turretSubsystem);
    }

    public TurretAutoAimVision(TurretSubsystem turretSubsystem, DoubleSupplier yaw) {
        m_turretSubsystem = turretSubsystem;
        m_yaw = yaw;
        m_endOnFinish = false;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {

        if (m_yaw.getAsDouble() > TurretConstants.kVisionTolerance) {
            m_turretSubsystem.set(0.5);
        } else if (m_yaw.getAsDouble() < -TurretConstants.kVisionTolerance) {
            m_turretSubsystem.set(-0.5);
        } else {
            m_turretSubsystem.stop();
        }
    }

// @Override
// public boolean isFinished() {
//     return m_yaw.getAsDouble() < -TurretConstants.kVisionTolerance && m_yaw.getAsDouble() > TurretConstants.kVisionTolerance && m_endOnFinish;
// }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turretSubsystem.stop();
    }
}
