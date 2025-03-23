package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase; //.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel; //.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    // private final RelativeEncoder m_encoder;
    private final SparkMaxConfig m_config;
    
    private final ProfiledPIDController m_pidController = new ProfiledPIDController(
        ArmConstants.kArmkP, 
        ArmConstants.kArmkI, 
        ArmConstants.kArmkD, 
        new TrapezoidProfile.Constraints(
            ArmConstants.kArmMaxAcceleration,
            ArmConstants.kArmMaxVelocity));
    private final ArmFeedforward m_feedforward = new ArmFeedforward(0, ArmConstants.kArmkGrav, 0);

    // Arm parameters
    private static final double MIN_ANGLE = ArmConstants.kArmMinAngle; // Degrees
    private static final double MAX_ANGLE = ArmConstants.kArmMaxAngle; // Degrees
    
    public ArmSubsystem() {
        m_motor = new SparkMax(ArmConstants.kArmMotorID, SparkLowLevel.MotorType.kBrushless);

        m_config = new SparkMaxConfig();        
        
        // Set soft limits
        // Configure soft limits (in rotations)
        // m_config.softLimit.forwardSoftLimitEnabled(true);
        // m_config.softLimit.reverseSoftLimitEnabled(true);
        // m_config.softLimit.forwardSoftLimit(MAX_ANGLE); // Max position
        // m_config.softLimit.reverseSoftLimit(MIN_ANGLE); // Min position

        // Configure encoder conversion factor (degrees per rotation)
        m_config.encoder.positionConversionFactor(360.0 / ArmConstants.kArmGearing);

        // Apply configuration        
        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_motor.getEncoder().setPosition(0);
        // Reset encoder to starting position
        // resetEncoder();
    }

    // public Command setPosition(double position) {
    //     SmartDashboard.putNumber("Set Arm Position: ", position);
    //     return run(() -> setAngle(position));
    // }

    // public void setAngle(double targetDegrees) {
    //     SmartDashboard.putNumber("Set Arm Angle", targetDegrees);
    //     double currentPosition = m_encoder.getPosition();
    //     double pidOutput = m_pidController.calculate(currentPosition, targetDegrees);
    //     double feedforwardOutput = m_feedforward.calculate(
    //         Math.toRadians(m_pidController.getSetpoint().position),
    //         m_pidController.getSetpoint().velocity);
            
    //     m_motor.setVoltage(pidOutput + feedforwardOutput);
    //     //m_motor.setVoltage(pidOutput);
    // }

    // public void resetEncoder() {
    //     m_encoder.setPosition(0);
    // }

    // public Command resetArm() {
    //     return run(() -> setAngle(MIN_ANGLE));
    // }

    public double getCurrentAngle() {
        return m_motor.getEncoder().getPosition();
    }
    double speed = 0;
    public Command stop() {
        return runOnce(() -> {
            m_motor.set(0.0);
            speed = 0;
        });
    }

    public Command rotateUp(){
        return runOnce(() -> {
            speed = 0.2;        });
    }

    public Command rotateDown(){
        return runOnce(() -> {
            speed = -0.2;
        });
    }

    @Override
    public void periodic() {
        System.out.print("encoder: " + getCurrentAngle());
        // Update safety controls
        // double currentAngle = getCurrentAngle();

        // if(currentAngle >= MAX_ANGLE || currentAngle <= MIN_ANGLE) {
        //     stop();
        // }
        if(getCurrentAngle() <-130 && speed < 0){
            speed = 0;
        }

        if(getCurrentAngle() >-10 && speed > 0){
            speed = 0;
        }
        System.out.println("Arm Speed: " +speed);
        m_motor.set(speed);
    }
}