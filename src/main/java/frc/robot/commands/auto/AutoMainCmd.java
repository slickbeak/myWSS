package frc.robot.commands.auto;

 import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.MoveRobot;
import frc.robot.commands.auto.RotateTest;
import frc.robot.subsystems.OmniDrive;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class AutoMainCmd extends SequentialCommandGroup
{   
	public AutoMainCmd()
    {
        super(
            // new MoveRobot(1, 1, 0, 0, 0.5),  
            // new MoveRobot(2, Math.PI, 0, 0, Math.PI),
            // new MoveRobot(1, 1, 0, 0, 0.5),
            // new MoveRobot(2, Math.PI, 0, 0, Math.PI)
            new MoveRobotSense(1 , 2 , 0 , 0, 0.5, ()->RobotContainer.m_sensor.getIRDistance()<21)
             );
    }
}
