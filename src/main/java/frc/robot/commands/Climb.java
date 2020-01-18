package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lift;

import java.util.function.BooleanSupplier;

public class Climb extends InstantCommand {

    private boolean extend = false;

    private Lift mLift;

    /**
     * Creates a new Climb command. The user can input whether or not they want to extend the arm.
     * If set to false, the arm will retract. Can be wrapped to implement smarter button operation.
     *
     * @param lift The subsystem used by this command.
     */
    public Climb(Lift lift, BooleanSupplier extend) {
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
        this.extend = extend.getAsBoolean();
        mLift = lift;

        addRequirements(lift);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (extend) mLift.extend();
        else mLift.retract();
    }
}

