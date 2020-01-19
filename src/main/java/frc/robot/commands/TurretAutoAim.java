package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.subsystems.Turret;

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

    public TurretAutoAim(Turret turret, DoubleSupplier driveTrainX, DoubleSupplier driveTrainY, DoubleSupplier driveTrainHeading, DoubleSupplier launchSpeed) {
        m_turret = turret;

        m_driveTrainX = driveTrainX;
        m_driveTrainY = driveTrainY;
        m_driveTrainHeading = driveTrainHeading;
        m_launchSpeed = launchSpeed;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_headingController.setTolerance(1);
        m_angleController.setTolerance(1);
    }

    @Override
    public void execute() {
        // Finds displacements for turret coordinates to goal
        double dx = (m_driveTrainX.getAsDouble() + Math.cos(TurretConstants.kAngleFromRobot + m_driveTrainHeading.getAsDouble()) * TurretConstants.kDistanceFromRobot) + TurretConstants.goalPosition.getX();
        double dy = (m_driveTrainY.getAsDouble() + Math.sin(TurretConstants.kAngleFromRobot  + m_driveTrainHeading.getAsDouble()) * TurretConstants.kDistanceFromRobot) + TurretConstants.goalPosition.getY();
        double dh = Math.sqrt(dx*dx + dy*dy);

        // Angle between the turret point and the goal point on the field
        double turretHeadingToGoal = new Rotation2d(dx, dy).getDegrees();

        // Angle between drive train heading and goal
        double DriveTrainToGoalAngle = Rotation2d.fromDegrees(turretHeadingToGoal).minus(Rotation2d.fromDegrees(m_driveTrainHeading.getAsDouble())).getDegrees();

        // Boolean that tests whether the turret is in range of the goal
        boolean notInRange = 180 - TurretConstants.HeaderConstants.kRange/2 <= Math.abs(DriveTrainToGoalAngle); // Note: turret is located on the back of the drive train

        double turretHeading = (notInRange) ? turretHeadingToGoal
                : (Rotation2d.fromDegrees(turretHeadingToGoal).plus(
                        Rotation2d.fromDegrees(TurretConstants.HeaderConstants.kRange - 2 * DriveTrainToGoalAngle))).getDegrees();

        double turretToGoalHeight = TurretConstants.goalHeight - TurretConstants.turretHeight;

        double turretAngle = (Math.acos((PhysicsConstants.kG * dh * dh + turretToGoalHeight)/(Math.sqrt(turretToGoalHeight * turretToGoalHeight + dh * dh))) + Math.atan(dh / turretToGoalHeight)) / 2;

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

