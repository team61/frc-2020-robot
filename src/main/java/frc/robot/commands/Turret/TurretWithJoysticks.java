package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretWithJoysticks extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private Timer m_timer = new Timer();

    private double m_prevTime;

    private DoubleSupplier m_speed;

    public TurretWithJoysticks(TurretSubsystem turretSubsystem, DoubleSupplier speed) {
        m_turretSubsystem = turretSubsystem;
        m_speed = speed;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        m_timer.reset();
        m_timer.start();
    }


    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;
        double speed = m_speed.getAsDouble();
        m_turretSubsystem.setVoltage(speed * TurretConstants.kMaxVoltage);

        double velocity = m_turretSubsystem.getEncoderRate();
        double acceleration = velocity / dt;
//        System.out.println(
//                "Velocity: " + velocity
//                        + "Acceleration: " + acceleration);

        m_prevTime = curTime;
        System.out.println(speed);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turretSubsystem.stop();
    }
}