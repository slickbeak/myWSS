package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.MoveRobot;;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class MoveRight_Return extends SequentialCommandGroup
{
    public MoveRight_Return()
    {
        super(
            new MoveRobot(2,-(Math.PI/2),0,0,Math.PI),
            new MoveRobotSense(1 , 0.5 , 0 , 0, 0.5, ()->RobotContainer.m_sensor.getIRDistance()<=20),
            new MoveRobot(2,-Math.PI,0,0,Math.PI),
            new MoveRobotSense(1 , 0.5 , 0 , 0, 0.5, ()->RobotContainer.m_sensor.getIRDistance()<=20),
            new MoveRobot(2,-(Math.PI/2),0,0,Math.PI)
        );
    }
}
