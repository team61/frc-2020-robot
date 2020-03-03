package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Feed.Feed;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Fire extends ParallelRaceGroup {
    public Fire(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
        addCommands(new Shoot(shooterSubsystem), new WaitCommand(Constants.FeederConstants.kFeederDelay).andThen(new Feed(feederSubsystem)));
    }
}
