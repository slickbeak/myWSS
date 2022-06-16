package frc.robot.subsystems;

//Java imports

//Vendor imports

import com.studica.frc.Cobra;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sensor extends SubsystemBase
{
    //Creates all necessary hardware interface here for sensors
    //For servo testing also????

    // Sensors
    private final DigitalInput input10;
    private final AnalogInput sharp22;
    private final AnalogInput sharp23;
    private int i; // for debugging


    // Good for debugging
    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Sensors");
    private final NetworkTableEntry D_inputDisp = tab.add("inputDisp", 0).getEntry();
    private final NetworkTableEntry D_cntDisp = tab.add("cntDisp", 0).getEntry();
    private final NetworkTableEntry A_distance22Disp = tab.add("distance22Disp", 0).getEntry();
    private final NetworkTableEntry A_distance23Disp = tab.add("distance23Disp", 0).getEntry();

    //Subsystem for sensors
    //This is just an example.
    public Sensor() {
        
        input10 = new DigitalInput(10);
        sharp22 = new AnalogInput(0);
        sharp23 = new AnalogInput(1);

    }

    public Boolean getSwitch() {
        return input10.get();
    }


    /**
     * Call for the raw ADC value
     * <p>
     * 
     * @param channel range 0 - 3 (matches what is on the adc)
     * @return value between 0 and 2047 (11-bit)
     */
    public int getCobraRawValue(final int channel) {
        return 0;
    }


    /**
     * Call for the distance measured by the Sharp IR Sensor
     * <p>
     * 
     * @return value between 0 - 100 (valid data range is 10cm - 80cm)
     */
    public double getIRDistance() {
        return (Math.pow(sharp22.getAverageVoltage(), -1.2045)) * 27.726;
    }

    public double getIRDistance2() {
        return (Math.pow(sharp23.getAverageVoltage(), -1.2045)) * 27.726;
    }
   
    /**
     * Code that runs once every robot loop
     */
    @Override
    public void periodic()
    {
        //Display on shuffleboard
        //These display is good for debugging but may slow system down.
        //Good to remove unnecessary display during competition
        i++;
        D_inputDisp.setBoolean(getSwitch());
        D_cntDisp.setNumber(i);
        A_distance22Disp.setNumber(getIRDistance());
        A_distance23Disp.setNumber(getIRDistance2());
    }
}