package frc.robot.subsystems;

//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class GripperSubsystem extends SubsystemBase {

    // Initialize your subsystem here
    public GripperSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        super.periodic();

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // @Override
    // public double getMeasurement() {
    //     return analogPotentiometer1.get();    
    // }

    // @Override
    // public void useOutput(double output, double setpoint) {
    //     output += setpoint*kF;
        
    //     gripperMC.set(output);
    // }
}