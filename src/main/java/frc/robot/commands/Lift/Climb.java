package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LiftSubsystem;

public class Climb extends InstantCommand {

    private LiftSubsystem m_liftSubsystem;

    /**
     * Creates a new Climb command. The user can input whether or not they want to extend the arm.
     * If set to false, the arm will retract. Can be wrapped to implement smarter button operation.
     *
     * @param liftSubsystem The subsystem used by this command.
     */
    public Climb(LiftSubsystem liftSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
        m_liftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_liftSubsystem.getToggle()) {
            m_liftSubsystem.retract();
        } else {
            m_liftSubsystem.extend();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_liftSubsystem.toggle();
    }
}

