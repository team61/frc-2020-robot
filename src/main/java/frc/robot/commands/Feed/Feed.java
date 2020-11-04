package frc.robot.commands.Feed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

import java.util.function.DoubleSupplier;

public class Feed extends CommandBase {

    private FeederSubsystem m_feederSubsystem;
    private DoubleSupplier speed;
    public Feed(FeederSubsystem feederSubsystem, DoubleSupplier speed) {
        m_feederSubsystem = feederSubsystem;
        this.speed = speed;

        addRequirements(feederSubsystem);
    }

    public Feed(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;
        speed = () -> 1;
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
        m_feederSubsystem.set(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
        m_feederSubsystem.setNumPowerCells(0);
    }

}
