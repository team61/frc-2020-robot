package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Dump extends CommandBase {

    private FeederSubsystem m_feederSubsystem;

    public Dump(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < FeederConstants.kSolenoidPorts.length; i++) {
            m_feederSubsystem.setSolenoidState(i, true);
        }
    }

    @Override
    public void execute() {
        m_feederSubsystem.setVoltage(-FeederConstants.kFeederSpeedVoltage);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
    }
}
