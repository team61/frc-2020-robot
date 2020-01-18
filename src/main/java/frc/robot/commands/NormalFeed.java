package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;

public class NormalFeed extends CommandBase {

    private Feeder m_feeder;


    public NormalFeed(Feeder feeder) {
        m_feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        m_feeder.set(FeederConstants.kFeederSpeedPer);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_feeder.isSwitchSet();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
    }
}
