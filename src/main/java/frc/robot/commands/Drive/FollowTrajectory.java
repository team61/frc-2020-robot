package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

public class FollowTrajectory extends CommandBase {

    private DriveSubsystem m_driveSubsystem;

    private RamseteController m_follower = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);

    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);

    private PIDController m_leftController = new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD);
    private PIDController m_rightController = new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD);

    private final Timer m_timer = new Timer();

    private Trajectory m_trajectory;

    private double m_prevTime;

    private DifferentialDriveWheelSpeeds m_prevSpeeds;

    public FollowTrajectory(Trajectory trajectory, DriveSubsystem driveSubsystem) {
        m_trajectory = trajectory;
        m_driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        Trajectory.State initialState = m_trajectory.sample(0);
        m_prevSpeeds = AutoConstants.kDriveKinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        DifferentialDriveWheelSpeeds targetWheelSpeeds = AutoConstants.kDriveKinematics.toWheelSpeeds(
                m_follower.calculate(m_driveSubsystem.getPose2d(), m_trajectory.sample(curTime)));

        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedForward = m_feedForward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
        double rightFeedForward = m_feedForward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

        double leftPID = m_leftController.calculate(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);
        double rightPID = m_rightController.calculate(m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

        double leftOutput = MathUtil.clamp(leftFeedForward + leftPID, -AutoConstants.kMaxVoltage, AutoConstants.kMaxVoltage);
        double rightOutput = MathUtil.clamp(rightFeedForward + rightPID, -AutoConstants.kMaxVoltage, AutoConstants.kMaxVoltage);

        m_driveSubsystem.tankDriveVolts(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopTankDrive();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}
