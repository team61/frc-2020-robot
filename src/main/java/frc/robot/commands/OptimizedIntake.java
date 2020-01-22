package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class OptimizedIntake extends CommandBase {

    private Intake m_intake;

    public OptimizedIntake(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.resetEncoder();
    }

    @Override
    public void execute() {
        m_intake.setSpeed(IntakeConstants.kIntakeSpeedRPM);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.resetController();
        new FunctionalCommand(() -> {},() -> m_intake.setSpeed(0), (i) -> m_intake.resetController(), m_intake::atSetpoint, m_intake).schedule();
    }
}
