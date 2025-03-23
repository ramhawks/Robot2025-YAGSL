package frc.robot.subsystems;


// import au.grapplerobotics.LaserCan;
// import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
// import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
// import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
// import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.RobotMath.Elevator;

//import static au.grapplerobotics.interfaces.LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
import static edu.wpi.first.units.Units.*;

import java.util.concurrent.ExecutionException;

public class ElevatorSubsystem extends SubsystemBase {
    //setUp
    private final DCMotor m_elevatorGearbox = DCMotor.getCIM(1);
    private SparkMax m_motor = new SparkMax(ElevatorConstants.kElevatorMotorID, SparkLowLevel.MotorType.kBrushed);
    private final RelativeEncoder m_encoder = m_motor.getEncoder(); // not tracked b/c brushed motor
    private final PIDController m_controller = new PIDController(
            ElevatorConstants.kElevatorKp,
            ElevatorConstants.kElevatorKi,
            ElevatorConstants.kElevatorKd);

    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);
    private ElevatorSim m_elevatorSim = null;
    
    // Sensors
    // private final LaserCan m_elevatorLaserCan = new LaserCan(0);
    // private final LaserCanSim m_elevatorLaserCanSim = new LaserCanSim(0);
    // private final RegionOfInterest m_laserCanROI = new RegionOfInterest(0, 0, 16, 16);
    // private final TimingBudget m_laserCanTimingBudget = TimingBudget.TIMING_BUDGET_20MS;
    private final Alert m_laserCanFailure = new Alert("LaserCAN failed to configure.", AlertType.kError);
    private final DigitalInput m_limitSwitchLow = new DigitalInput(2);
    private final DigitalInput topLimitSwitch = new DigitalInput(1);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(0);
    private boolean m_isHomed = false;
    private final double circumference = 2 * Math.PI * ElevatorConstants.kElevatorDrumRadius;

    private double overridespeed = 0;
    private boolean overridePid = false;

    public ElevatorSubsystem() {
        // Reset controller with initial position
        resetController();
     
        //SparkMaxConfig config = new SparkMaxConfig();
        //config.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
          //  .closedLoopRampRate(ElevatorConstants.kElevatorRampRate).closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //.outputRange(-1, 1);
            //.openLoopRampRate(ElevatorConstants.kElevatorRampRate);
        //m_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSim(m_elevatorGearbox,
                    ElevatorConstants.kElevatorGearing,
                    ElevatorConstants.kElevatorCarriageMass,
                    ElevatorConstants.kElevatorDrumRadius,
                    ElevatorConstants.kElevatorMinHeightMeters,
                    ElevatorConstants.kElevatorMaxHeightMeters,
                    true,
                    0.0,
                    0.02,
                    0.0);
            SmartDashboard.putData("Elevator Low limit Switch", m_limitSwitchLow);
        }

        try {
            // m_elevatorLaserCanSim.setRangingMode(RangingMode.LONG);
        } catch (Exception e) {
            m_laserCanFailure.set(true);
        }
    }

    private void resetController() {
       // m_controller.reset(m_encoder.getPosition(), m_encoder.getVelocity());
        SmartDashboard.putString("Reset Controller", "True");
    }

    // Homing command
    public Command homeElevator() {
        SmartDashboard.putBoolean("Home Elevator", m_isHomed);
        return Commands.runOnce(() -> {
            // Reset the controller before going to a new goal. Do we need to do this? 
            resetController();
            // Move down until limit switch triggers
            m_motor.set(-0.2);
            }).until(() -> bottomLimitSwitch.get()) //m_motor.getReverseLimitSwitch().isPressed()
            .andThen(() -> {
              m_motor.stopMotor();
              m_motor.getEncoder().setPosition(0);
              m_isHomed = true;
              SmartDashboard.putBoolean("Home Elevator", m_isHomed);
          }          
        );
    }

    /*
     * Method is called periodically by the CommandScheduler to update subsystem-specific tasks in the simulaton
     */
    public void simulationPeriodic() {
        //set input(voltage)
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        //update-every 20 milliseconds
        m_elevatorSim.update(0.02);

        m_motorSim.iterate(
                Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
                        .per(Second)
                        .in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

        // Update lasercan sim.
        // m_elevatorLaserCanSim.setMeasurementFullSim(new Measurement(
        //         LASERCAN_STATUS_VALID_MEASUREMENT,
        //         (int) (Math.floor(Meters.of(m_elevatorSim.getPositionMeters()).in(Millimeters)) +
        //                 ElevatorConstants.kLaserCANOffset.in(Millimeters)),
        //         0,
        //         true,
        //         m_laserCanTimingBudget.asMilliseconds(),
        //         m_laserCanROI
        // ));

        //comment out for arm demo
        //Constants.elevatorMech.setLength(getPositionMeters());
        //Constants.elevatorCarriage.setPosition(AlgaeArmConstants.kAlgaeArmLength, getPositionMeters());
    }

    public double getPositionMeters() {
        return (m_encoder.getPosition() / ElevatorConstants.kElevatorGearing) * circumference;
    }

    // public double getVelocityMetersPerSecond() {
    //     return ((m_encoder.getVelocity() / 60) / ElevatorConstants.kElevatorGearing) * circumference;
    // }

    // public void reachGoal(double goal){
    //     double getControllerSetpointVelocity = m_controller.getSetpoint().velocity;
    //     double feedForwardCalcWithVelocity = m_feedForward.calculateWithVelocities(
    //         getVelocityMetersPerSecond(), 
    //         getControllerSetpointVelocity);
    //     double getNextOutputOfPID = m_controller.calculate(getPositionMeters(), goal);

    //     double voltsOutput = MathUtil.clamp(
    //         feedForwardCalcWithVelocity + getNextOutputOfPID, 
    //         -7, 
    //         7);
    //     m_motor.setVoltage(voltsOutput);
    // }

    /*
     * Set voltage to a certain amount, our goal and run it for Command-based programming
     */
    public Command setGoal(double goal){
        return runOnce(() -> {
            m_controller.setSetpoint(goal);
            SmartDashboard.putBoolean("Elevator Height Reached", true);
        });
    }

    /*
     * Returns a setGoal. Uses a condition of around height to stop
     * Param: height is in meters
     */
    // public Command setElevatorHeight(double height){
    //     SmartDashboard.getNumber("Set Elevator Height", height);
    //     resetController();
    //     return setGoal(height).until(() -> aroundHeight(height));
    // }

    // public boolean aroundHeight(double height){
    //     return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    // }
    // public boolean aroundHeight(double height, double tolerance){
    //     return MathUtil.isNear(height, getPositionMeters(), tolerance);
    // }

     /**
     * Stop the control loop and motor output.
     */
    public Command stop() {
        return runOnce(() -> {
            m_motor.set(0.0);
            overridespeed = 0;
            overridePid = false;
        });
    }

    public Command driveUp(){
        return runOnce(() -> {
            overridespeed = Constants.ElevatorConstants.kMaxVelocity;
            overridePid = true;
        });
    }

    public Command driveDown(){
        return runOnce(() -> {
            overridespeed = -Constants.ElevatorConstants.kMaxVelocity/2;
            overridePid = true;
        });
    }
    
    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry() {
    }

    @Override
    public void periodic() {
        System.out.println("up: " + topLimitSwitch.get() + " down: " + bottomLimitSwitch.get());
        double speed = m_controller.calculate(getPositionMeters());
        
        if(overridePid)
            speed = overridespeed;

        if(topLimitSwitch.get() && speed >0)
            speed = 0;

        if(bottomLimitSwitch.get() && speed < 0)
            speed = 0;
            
        System.out.println("Speed: " +speed);
        m_motor.set(speed);
    }
}