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
public class MoveArm extends CommandBase
{
    //Grab the subsystem instance from RobotContainer
    private final static Arm m_arm = RobotContainer.m_arm;

    private boolean m_endFlag = false;
    private double[] m_angle;

    // Servo0
    private double dT0 = 0.02;
    private TrapezoidProfile.Constraints m_constraints0;
    private TrapezoidProfile.State m_goal0 = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint0 = new TrapezoidProfile.State();
    private TrapezoidProfile m_profile0;

    private double start_pos0;
    private final double tgt_pos0;

    // Servo1
    private double dT1 = 0.02;
    private TrapezoidProfile.Constraints m_constraints1;
    private TrapezoidProfile.State m_goal1 = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint1 = new TrapezoidProfile.State();
    private TrapezoidProfile m_profile1;

    private double start_pos1;
    private final double tgt_pos1;


    /**
     * This command moves the robot a certain distance following a trapezoidal speed profile.
     * <p>
     * 
     * @param position - where to rotate to
     * @param maxSpeed - max speed of robot
     */
    //This move the robot a certain distance following a trapezoidal speed profile.
    public MoveArm(double pos0, double pos1, double maxSpeed0, double maxSpeed1)
    {
        m_angle = m_arm.GotoPosXY(pos0, pos1);
        tgt_pos0 = m_angle[0];
        tgt_pos1 = m_angle[1];
        m_constraints0 = new TrapezoidProfile.Constraints(maxSpeed0, 2.0*Math.PI);
        m_constraints1 = new TrapezoidProfile.Constraints(maxSpeed1, 2.0*Math.PI);
        //addRequirements(m_drive); // Adds the subsystem to the command
        
    }

    /**
     * Runs before execute
     */
    @Override
    public void initialize()
    {   
        start_pos0 = m_arm.getServoAngle0();
        start_pos1 = m_arm.getServoAngle1();

        m_goal0 = new TrapezoidProfile.State(tgt_pos0, 0);
        m_setpoint0 = new TrapezoidProfile.State(start_pos0, 0);
        m_goal1 = new TrapezoidProfile.State(tgt_pos1, 0);
        m_setpoint1 = new TrapezoidProfile.State(start_pos1, 0);
        
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
         m_profile0 = new TrapezoidProfile(m_constraints0, m_goal0, m_setpoint0);
         m_setpoint0 = m_profile0.calculate(dT0);
         m_arm.setServoAngle0(m_setpoint0.position);

         m_profile1 = new TrapezoidProfile(m_constraints1, m_goal1, m_setpoint1);
         m_setpoint1 = m_profile1.calculate(dT1);
         m_arm.setServoAngle1(m_setpoint1.position);
 
         if ((m_profile0.isFinished(dT0) && m_profile1.isFinished(dT1)) ) {
             //distance reached or end condition met. End the command
             //This class should be modified so that the profile can end on other conditions like
             //sensor value etc.
             m_arm.setServoAngle0(m_setpoint0.position);
             m_arm.setServoAngle1(m_setpoint1.position);
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