package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Good to create a subsystem for robot arm
public class Arm extends SubsystemBase {
    private final Servo servo;
    private double ServoValue;

    // Good for debugging
    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Sensors");
    private final NetworkTableEntry D_armValue = tab.add("armValue", 0).getEntry();

    public Arm () {
        servo = new Servo(8);
    }

    /**
     * Sets the servo angle
     * <p>
     * 
     * @param degrees degree to set the servo to, range 0° - 300°
     */
    public void setServoAngle(final double degrees){
        ServoValue = degrees;
        servo.setAngle(degrees);
    }

    @Override
    public void periodic()
    {
        D_armValue.setDouble(ServoValue);
    }

    
}
