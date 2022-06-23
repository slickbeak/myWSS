package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.MoveRobot;
import frc.robot.commands.auto.RotateTest;
/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class AutoMainCmd extends SequentialCommandGroup
{   
    private double dist = RobotContainer.m_sensor.getIRDistance();
    private boolean sw_state = RobotContainer.m_sensor.getSwitch2();
    public static int cycle = 0;

	public AutoMainCmd()
    {
        super(
            new MoveArm(0,0.5,10,10)

            // new MoveRobotSense(1 , 0.5 , 0 , 0, 0.5, ()->RobotContainer.m_sensor.getIRDistance()<=20),
            // new LoopCmd(new CheckObstacle(), ()->(RobotContainer.m_sensor.getSwitch2()==false || cycle == 5)),
            // new MoveRobotSense(1 , -0.5 , 0 , 0, 0.5, ()->RobotContainer.m_sensor.getIRDistance()<=20),

            // new MoveRobot(1, 1, 0, 0, 0.5),  
            // new MoveRobot(2, Math.PI, 0, 0, Math.PI),
            // new MoveRobot(1, 1, 0, 0, 0.5),
            // new MoveRobot(2, Math.PI, 0, 0, Math.PI)
            // new MoveRobotSense(1 , 1 , 0 , 0, 0.5, ()->RobotContainer.m_sensor.getIRDistance()<=20),
            // new SelectCommand(
            //     Map.ofEntries(
            //         Map.entry(CommandSelector.ONE, new MoveRobot(2,-(Math.PI/2),0,0,Math.PI)),
            //         Map.entry(CommandSelector.TWO, new MoveRobot(2,(Math.PI/2),0,0,Math.PI)),
            //         Map.entry(CommandSelector.THREE, new MoveRobot(2,Math.PI,0,0,Math.PI))
            //          ),
            //     AutoMainCmd::selectCmd123
            // )
            // new MoveServo(15,0,0,20),
            // new MoveServo(55,0,0,20),
            // new MoveServo(105,0,0,10)
        );
        
    }
}
