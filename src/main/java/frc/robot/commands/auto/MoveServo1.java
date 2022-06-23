package frc.robot.commands.auto;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//WPI imports
import edu.wpi.first.wpilibj2.command.CommandBase;
//RobotContainer import
import frc.robot.RobotContainer;

//Subsystem imports
import frc.robot.subsystems.Arm;

/**
 * SimpleDrive class
 * <p>
 * This class drives a motor 
 */
public class MoveServo1 extends CommandBase
{
    //Grab the subsystem instance from RobotContainer
    private final static Arm m_arm = RobotContainer.m_arm;
    private double dT = 0.02;
    private boolean m_endFlag = false;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile m_profile;

    private double start_pos;
    private final double tgt_pos;
    private final double _startSpeed;
    private final double _endSpeed;


    /**
     * This command moves the robot a certain distance following a trapezoidal speed profile.
     * <p>
     * 
     * @param position - where to rotate to
     * @param maxSpeed - max speed of robot
     */
    //This move the robot a certain distance following a trapezoidal speed profile.
    public MoveServo1(double angle, double startSpeed, double endSpeed, double maxSpeed)
    {
        tgt_pos = angle;
        _startSpeed = startSpeed;
        _endSpeed = endSpeed;
        m_constraints = new TrapezoidProfile.Constraints(maxSpeed, 2.0*Math.PI);
           
        //addRequirements(m_drive); // Adds the subsystem to the command
        
    }

    /**
     * Runs before execute
     */
    @Override
    public void initialize()
    {   
        start_pos = m_arm.getServoAngle1();

        m_goal = new TrapezoidProfile.State(tgt_pos, _endSpeed);
        m_setpoint = new TrapezoidProfile.State(start_pos, _startSpeed);
        
        m_endFlag = false;
    }
    /**
     * Condition to end speed profile
     */
    public boolean endCondition()
    {
        return false;
    }
    /**
     * Called continously until command is ended
     */
    @Override
    public void execute()
    {
         //Create a new profile to calculate the next setpoint(speed) for the profile
         m_profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
         m_setpoint = m_profile.calculate(dT);
         m_arm.setServoAngle1(m_setpoint.position);
 
         if ((m_profile.isFinished(dT)) ) {
             //distance reached or end condition met. End the command
             //This class should be modified so that the profile can end on other conditions like
             //sensor value etc.
             m_arm.setServoAngle1(m_setpoint.position);
             m_endFlag = true;
         }
    }

    /**
     * Called when the command is told to end or is interrupted
     */
    @Override
    public void end(boolean interrupted)
    {

    }

    /**
     * Creates an isFinished condition if needed
     */
    @Override
    public boolean isFinished()
    {
        return m_endFlag;
    }

}