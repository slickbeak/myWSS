package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

// Good to create a subsystem for robot arm
public class Arm extends SubsystemBase {
    private final Servo servo0;
    private final Servo servo1;
    private double angle_0;
    private double angle_1;

    // Good for debugging
    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private final NetworkTableEntry D_armValue0 = tab.add("armValue0", 0).getEntry();
    private final NetworkTableEntry D_armValue1 = tab.add("armValue1", 0).getEntry();
    private final NetworkTableEntry D_armTarget0 = tab.add("targetValue0", 0).getEntry();
    private final NetworkTableEntry D_armTarget1 = tab.add("targetValue1", 0).getEntry();
    private final NetworkTableEntry D_navYaw = tab.add("Nav Yaw", 0).getEntry();
    private final NetworkTableEntry D_curHeading = tab.add("curHeading", 0).getEntry();
    private final NetworkTableEntry D_tgtHeading = tab.add("tgtHeading", 0).getEntry();

    public Arm () {
        servo0 = new Servo(8);
        servo1 = new Servo(9);
        servo0.setAngle(90);
        servo1.setAngle(0);
    }

    /**
     * Sets the servo angle
     * <p>
     * 
     * @param degrees degree to set the servo to, range 0° - 300°
     */
    public void setServoAngle0(final double degrees){
        servo0.setAngle(degrees);
    }

    public double getServoAngle0(){
        return servo0.getAngle();
    }

    public void setServoAngle1(final double degrees){
        servo1.setAngle(degrees);
    }

    public double getServoAngle1(){
        return servo1.getAngle();
    }

    public double[] GotoPosXY(double x, double y){
        int servo0_dir, servo1_dir;
        if (x>y){
            angle_1 = Math.acos(((x*x) + (y*y) - (0.25*0.25) - (0.25*0.25))/(2*0.25*0.25));
            angle_0 = Math.atan(y/x) - Math.atan((0.25*Math.sin(angle_1))/(0.25+(0.25*(Math.cos(angle_1)))));
        }
        else{
            angle_1 = -Math.acos(((x*x) + (y*y) - (0.25*0.25) - (0.25*0.25))/(2*0.25*0.25));
            angle_0 = Math.atan(y/x) + Math.atan((0.25*Math.sin(angle_1))/(0.25+(0.25*(Math.cos(angle_1)))));
        }
        servo0_dir = (angle_0>0)?1:-1;
        servo1_dir = (angle_1>0)?1:-1;
        angle_0 *= servo0_dir;
        angle_1 *= servo1_dir;
        if(servo0_dir == -1 && angle_0 > 180)
            angle_0 = (2*Math.PI) - angle_0;
        if(servo1_dir == -1 && angle_1 > 180)
            angle_1 = (2*Math.PI) - angle_1;
        angle_0 = Math.toDegrees(angle_0);
        angle_1 = Math.toDegrees(angle_1);
        
        return new double[]{angle_0, angle_1};
    }


 
    @Override
    public void periodic()
    {
        D_armValue0.setDouble(servo0.getAngle());
        D_armValue1.setDouble(servo1.getAngle());
        D_armTarget0.setDouble(angle_0);
        D_armTarget1.setDouble(angle_1);
    }

    
}
