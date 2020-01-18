package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class NormalIntake extends CommandBase {

    private Intake m_intake;

    public NormalIntake(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.set(IntakeConstants.kIntakeSpeedPer);
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
    }
}
