package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lift;

public class Climb extends InstantCommand {

    private boolean m_extend = false;

    private Lift m_lift;

    /**
     * Creates a new Climb command. The user can input whether or not they want to extend the arm.
     * If set to false, the arm will retract. Can be wrapped to implement smarter button operation.
     *
     * @param lift The subsystem used by this command.
     */
    public Climb(Lift lift, boolean extend) {
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
        m_extend = extend;
        m_lift = lift;

        addRequirements(lift);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_extend) m_lift.extend();
        else m_lift.retract();
    }
}

