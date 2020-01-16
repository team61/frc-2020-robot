/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import lib.components.LogitechJoystick;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveTrain m_driveTrain = DriveTrain.getInstance();

    private final LogitechJoystick jLeft = new LogitechJoystick(OIConstants.jLeft);
    private final LogitechJoystick jRight = new LogitechJoystick(OIConstants.jRight);

    private final Command m_autoCommand = null;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_driveTrain.setDefaultCommand(new TankDrive(m_driveTrain, jLeft::getYAxis, jRight::getYAxis));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() throws IOException {
        // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(AutoConstants.kS,
                                AutoConstants.kV,
                                AutoConstants.kA),
                        AutoConstants.kDriveKinematics,
                        AutoConstants.kMaxVoltage);

        // Create config for trajectory
        TrajectoryConfig config =
                // Add constraints to trajectory
                new TrajectoryConfig(AutoConstants.kMaxVelocity,
                        AutoConstants.kMaxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(AutoConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config
        );

        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/TestPath.wpilib.json"));

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                m_driveTrain::getPose2d,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(AutoConstants.kS,
                        AutoConstants.kV,
                        AutoConstants.kA),
                AutoConstants.kDriveKinematics,
                m_driveTrain::getWheelSpeeds,
                new PIDController(AutoConstants.kP, 0, 0),
                new PIDController(AutoConstants.kP, 0, 0),
                // RamseteCommand passes volts to the callback
                m_driveTrain::tankDriveVolts,
                m_driveTrain
        );

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(m_driveTrain::stopTankDrive);
    }
}
