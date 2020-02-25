package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

public class DriveForDistance extends CommandBase {

    private DriveSubsystem m_driveSubsystem;

    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);

    private ProfiledPIDController m_leftController = new ProfiledPIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD, AutoConstants.kConstraints);
    private ProfiledPIDController m_rightController = new ProfiledPIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD, AutoConstants.kConstraints);

    private final Timer m_timer = new Timer();

    private double m_prevTime;

    private double m_goal;

    public DriveForDistance(DriveSubsystem driveSubsystem, double goal) {
        m_driveSubsystem = driveSubsystem;
        m_goal = goal;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_driveSubsystem.resetEncoders();
        m_leftController.setGoal(m_goal);
        m_rightController.setGoal(m_goal);

        m_leftController.setTolerance(0.2);
        m_rightController.setTolerance(0.2);
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        double leftSpeedSetpoint = m_leftController.getSetpoint().velocity;
        double rightSpeedSetpoint = m_rightController.getSetpoint().velocity;

        double leftFeedForward = m_feedForward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - m_driveSubsystem.getLeftEncoderRate()) / dt);
        double rightFeedForward = m_feedForward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - m_driveSubsystem.getRightEncoderRate()) / dt);

        double leftPID = m_leftController.calculate(m_driveSubsystem.getLeftEncoderDistance());
        double rightPID = m_rightController.calculate(m_driveSubsystem.getRightEncoderDistance());

        double leftOutput = MathUtil.clamp(leftFeedForward + leftPID, -AutoConstants.kMaxVoltage, AutoConstants.kMaxVoltage);
        double rightOutput = MathUtil.clamp(rightFeedForward + rightPID, -AutoConstants.kMaxVoltage, AutoConstants.kMaxVoltage);

        m_driveSubsystem.tankDriveVolts(leftOutput, rightOutput);

        m_prevTime = curTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopTankDrive();
    }

    @Override
    public boolean isFinished() {
        return m_leftController.atGoal() && m_rightController.atGoal();
    }
}
