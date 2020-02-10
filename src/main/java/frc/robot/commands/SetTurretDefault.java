package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretDefault extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private ProfiledPIDController m_controller = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kConstraints);
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);

    public SetTurretDefault(TurretSubsystem turretSubsystem) {
        m_turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        double profile = m_controller.calculate(m_turretSubsystem.getEncoderDistance(),TurretConstants.kDefaultState);
        double feedForward = m_feedForward.calculate(m_controller.getSetpoint().velocity);
        double output = profile + feedForward;
        m_turretSubsystem.setVoltage(output);
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