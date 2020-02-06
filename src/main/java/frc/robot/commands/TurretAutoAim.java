package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAutoAim extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private ProfiledPIDController m_controller = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD,TurretConstants.kConstraints);
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);

    private NetworkTableEntry yawEntry;

    public TurretAutoAim(TurretSubsystem turretSubsystem) {
        m_turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("chameleon-vision").getSubTable(TurretConstants.TurretCamName);
        yawEntry = table.getEntry("targetYaw");
    }

    @Override
    public void execute() {
        double yaw = yawEntry.getDouble(0);
        m_turretSubsystem.setVoltage(m_controller.calculate(yaw, 0) + m_feedforward.calculate(m_controller.getSetpoint().velocity));
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
