package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Intake;

public class ContinuousIntake extends CommandBase {
    private Intake m_intake;

    public ContinuousIntake(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.set(FeederConstants.kFeederSpeedPer);
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
