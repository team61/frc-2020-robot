package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LiftSubsystem;

public class Climb extends InstantCommand {

    private boolean m_extend = false;

    private LiftSubsystem m_liftSubsystem;

    /**
     * Creates a new Climb command. The user can input whether or not they want to extend the arm.
     * If set to false, the arm will retract. Can be wrapped to implement smarter button operation.
     *
     * @param liftSubsystem The subsystem used by this command.
     */
    public Climb(LiftSubsystem liftSubsystem, boolean extend) {
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
        m_extend = extend;
        m_liftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_extend) {
            m_liftSubsystem.extend();
        } else {
            m_liftSubsystem.retract();
        }
    }
}

