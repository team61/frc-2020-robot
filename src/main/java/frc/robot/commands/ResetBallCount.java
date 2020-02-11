package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FeederSubsystem;

public class ResetBallCount extends InstantCommand {

    private FeederSubsystem m_feederSubsystem;

    public ResetBallCount(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;
    }

    @Override
    public void execute() {
        m_feederSubsystem.setNumPowerCells(0);
    }
}
