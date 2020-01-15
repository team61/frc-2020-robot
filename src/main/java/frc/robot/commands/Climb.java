package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

import java.util.function.BooleanSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class Climb extends CommandBase {

    private boolean extend = false;

    private Lift mLift;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new Climb command. The user can input whether or not they want to extend the arm.
     * If set to false, the arm will retract. Can be wrapped to implement smarter button operation.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Climb(Lift lift, BooleanSupplier extend) {
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
        this.extend = extend.getAsBoolean();
        mLift = lift;

        addRequirements(lift);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (extend) mLift.extend();
        else mLift.retract();
        end(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

