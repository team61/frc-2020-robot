package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class TurretAutoAim extends CommandBase {

    private Turret m_turret;

    private ProfiledPIDController m_headingController = new ProfiledPIDController(
            TurretConstants.HeaderConstants.kP, TurretConstants.HeaderConstants.kI, TurretConstants.HeaderConstants.kD,
            TurretConstants.HeaderConstants.kConstraints, MiscellaneousConstants.kDt);

    private ProfiledPIDController m_angleController =  new ProfiledPIDController(
            TurretConstants.AngleConstants.kP, TurretConstants.AngleConstants.kI, TurretConstants.AngleConstants.kD,
            TurretConstants.AngleConstants.kConstraints, MiscellaneousConstants.kDt);

    private SimpleMotorFeedforward m_headingFeedforward = new SimpleMotorFeedforward(TurretConstants.HeaderConstants.kS, TurretConstants.HeaderConstants.kV, TurretConstants.HeaderConstants.kA);
    private ArmFeedforward m_angleFeedforward = new ArmFeedforward(TurretConstants.AngleConstants.kS, TurretConstants.AngleConstants.kCos, TurretConstants.AngleConstants.kV, TurretConstants.AngleConstants.kA);

    private DoubleSupplier m_driveTrainX;
    private DoubleSupplier m_driveTrainY;
    private DoubleSupplier m_driveTrainHeading;
    private DoubleSupplier m_launchSpeed;

    private DoubleConsumer m_setLaunchSpeed;

    public TurretAutoAim(Turret turret, DoubleSupplier driveTrainX, DoubleSupplier driveTrainY, DoubleSupplier driveTrainHeading, DoubleSupplier launchSpeed, DoubleConsumer setLaunchSpeed) {
        m_turret = turret;

        m_driveTrainX = driveTrainX;
        m_driveTrainY = driveTrainY;
        m_driveTrainHeading = driveTrainHeading;
        m_launchSpeed = launchSpeed;

        m_setLaunchSpeed = setLaunchSpeed;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_headingController.setTolerance(1);
        m_angleController.setTolerance(1);
    }

    @Override
    public void execute() {

        double driveTrainHeading = m_driveTrainHeading.getAsDouble();
        // Finds displacements for turret coordinates to goal
        double dx = (m_driveTrainX.getAsDouble() + Math.cos(TurretConstants.kAngleFromRobot + driveTrainHeading) * TurretConstants.kDistanceFromRobot) + TurretConstants.goalPosition.getX();
        double dy = (m_driveTrainY.getAsDouble() + Math.sin(TurretConstants.kAngleFromRobot  + driveTrainHeading) * TurretConstants.kDistanceFromRobot) + TurretConstants.goalPosition.getY();
        double dh = Math.sqrt(dx*dx + dy*dy);

        // Angle between the turret point and the goal point on the field
        double turretHeadingToGoal = new Rotation2d(dx, dy).getDegrees();

        // Angle between drive train heading and goal
        double DriveTrainToGoalAngle = Rotation2d.fromDegrees(turretHeadingToGoal).minus(Rotation2d.fromDegrees(driveTrainHeading)).getDegrees();

        // Boolean that tests whether the turret is in range of the goal
        boolean notInRange = 180 - TurretConstants.HeaderConstants.kRange/2 <= Math.abs(DriveTrainToGoalAngle); // Note: turret is located on the back of the drive train

        // Decides turretHeading based on whether in range of goal
        double turretHeading = (notInRange) ? turretHeadingToGoal
                // This heading moves closer to depending on how close a side is to the goal
                : (Rotation2d.fromDegrees(turretHeadingToGoal).plus(
                        Rotation2d.fromDegrees(TurretConstants.HeaderConstants.kRange - 2 * DriveTrainToGoalAngle))).getDegrees();

        m_setLaunchSpeed.accept((dh > LauncherConstants.kSlowDistanceRange) ? LauncherConstants.kFastSpeedRPM : LauncherConstants.kSlowSpeedRPM);

        double turretToGoalHeight = TurretConstants.goalHeight - TurretConstants.turretHeight;

        // Chooses ball speed depending on which speed is selected on the launcher
        double ballSpeed = (m_launchSpeed.getAsDouble() == LauncherConstants.kFastSpeedRPM) ? LauncherConstants.kFastBallSpeed : LauncherConstants.kSlowBallSpeed;

        // Kinematic formula to calculate angle of turret
        double turretAngle =
                (Math.acos(
                        ((PhysicsConstants.kG * dh * dh) / (ballSpeed * ballSpeed) + turretToGoalHeight)
                                /(Math.sqrt(turretToGoalHeight * turretToGoalHeight + dh * dh)))
                        + Math.atan(dh / turretToGoalHeight)) / 2;

        // Sets speed of heading and angle motors based on feedforward and pid controllers

        m_turret.setHeadingSpeed(
                m_headingFeedforward.calculate(
                        m_headingController.getSetpoint().position, m_headingController.getSetpoint().velocity)
                        + m_headingController.calculate(m_turret.getYaw(), turretHeading));

        m_turret.setAngleSpeed(
                m_angleFeedforward.calculate(
                        m_angleController.getSetpoint().position, m_angleController.getSetpoint().velocity)
                        + m_angleController.calculate(m_turret.getPitch(), turretAngle));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stopHeading();
        m_turret.stopAngle();
    }
}

