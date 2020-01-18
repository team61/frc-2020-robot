package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class OptimizedIntake extends CommandBase {

    private Intake m_intake;

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);
    private PIDController m_controller = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

    public OptimizedIntake(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.setVoltage(m_feedforward.calculate(IntakeConstants.kIntakeSpeedRPM, IntakeConstants.kMaxAcc) + m_controller.calculate(m_intake.getEncoderRate(), IntakeConstants.kIntakeSpeedRPM));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_controller.reset();
    }
}
