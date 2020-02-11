package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import lib.util.Pose2dSupplier;

public class TurretAutoAimOdometry extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private ProfiledPIDController m_controller = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD,TurretConstants.kConstraints);
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);

    private Pose2dSupplier m_pose2dSupplier;

    private Timer m_timer = new Timer();

    private double m_prevTime;

    public TurretAutoAimOdometry(TurretSubsystem turretSubsystem, Pose2dSupplier pose2dSupplier) {
        m_turretSubsystem = turretSubsystem;
        m_pose2dSupplier = pose2dSupplier;

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

        double speedSetpoint = m_controller.getSetpoint().velocity;

        Translation2d robotTranslation2d = m_pose2dSupplier.getAsPose2d().getTranslation();
        double robotX = robotTranslation2d.getX();
        double robotY = robotTranslation2d.getY();

        double goalX = TurretConstants.kGoalPosition.getX();
        double goalY = TurretConstants.kGoalPosition.getY();

        double degrees = Math.toDegrees(Math.atan2((goalY - robotY), (goalX - robotX)));

        double profile = m_controller.calculate(degrees, 0);
        double feedForward =  m_feedForward.calculate(speedSetpoint, (speedSetpoint - m_turretSubsystem.getEncoderRate()) / dt);

        double output = MathUtil.clamp(profile + feedForward, 0, TurretConstants.kMaxVoltage);

        m_turretSubsystem.setVoltage(output);

        m_prevTime = curTime;
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
