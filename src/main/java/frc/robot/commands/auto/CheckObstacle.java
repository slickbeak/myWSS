package frc.robot.commands.auto;

import java.util.List;
import java.util.Map;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.AutoMainCmd;
import frc.robot.commands.auto.MoveRobot;
import frc.robot.commands.auto.MoveForward_Return;
import frc.robot.commands.auto.MoveRight_Return;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class CheckObstacle extends SequentialCommandGroup
{
    private enum CommandSelector {
        ONE, TWO, THREE
    }

    static public CommandSelector selectCmd123() {
        if (RobotContainer.m_sensor.getIRDistance()<=50)
            return CommandSelector.ONE;
        else if (RobotContainer.m_sensor.getIRDistance()>50)
            return CommandSelector.TWO;
        else{
            return CommandSelector.THREE;
        }
    }

    public CheckObstacle()
    {
        super(
            new SelectCommand(
                Map.ofEntries(
                    Map.entry(CommandSelector.ONE, new MoveRight_Return()),
                    Map.entry(CommandSelector.TWO, new MoveForward_Return())
                    ),
                CheckObstacle::selectCmd123
            )
        );
        AutoMainCmd.cycle++;
        System.out.println("ENTER LOOP!");
    }
}
