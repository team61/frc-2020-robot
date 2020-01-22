package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;

public class OptimizedFeed extends CommandBase {

    private Feeder m_feeder;

    public OptimizedFeed(Feeder feeder) {
        m_feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        double feederSpeedRPM = 0;
        if (!m_feeder.isSwitchSet()) {
            feederSpeedRPM = FeederConstants.kFeederSpeedRPM;
        } else {
            feederSpeedRPM = 0;
        }
        m_feeder.setSpeed(feederSpeedRPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_feeder.resetController();
    }
}
