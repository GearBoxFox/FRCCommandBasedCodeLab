package com.gos.codelabs.basic_simulator.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SimableCANSparkMax;
import com.gos.codelabs.basic_simulator.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.BaseDigitalInputWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;

public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {
    public static final double ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(1);

    public enum Positions {
        LOW(Units.inchesToMeters(10)),
        MID(Units.inchesToMeters(35)),
        HIGH(Units.inchesToMeters(45));

        public final double m_heightMeters;

        Positions(double heightMeters) {
            m_heightMeters = heightMeters;
        }
    }

    private final PIDController m_pid;

    private final SimableCANSparkMax m_liftMotor;
    private final RelativeEncoder m_liftEncoder;
    private final DigitalInput m_lowerLimitSwitch;
    private final DigitalInput m_upperLimitSwitch;

    private ElevatorSimWrapper m_elevatorSim;

    private Mechanism2d m_mech;
    private MechanismRoot2d m_root;
    private MechanismLigament2d m_elevator;

    private static final class ElevatorSimConstants {
        public static final double K_ELEVATOR_GEARING = 10.0;
        public static final double K_CARRIAGE_MASS = 4.0; // kg
        public static final double K_MIN_ELEVATOR_HEIGHT = 0.0;
        public static final double K_MAX_ELEVATOR_HEIGHT = Units.inchesToMeters(50e50);
        public static final DCMotor K_ELEVATOR_GEARBOX = DCMotor.getVex775Pro(4);
        public static final double K_ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(2.0);
    }

    public ElevatorSubsystem() {
        m_liftMotor = new SimableCANSparkMax(Constants.CAN_LIFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_liftEncoder = m_liftMotor.getEncoder();

        m_lowerLimitSwitch = new DigitalInput(Constants.DIO_LIFT_LOWER_LIMIT);
        m_upperLimitSwitch = new DigitalInput(Constants.DIO_LIFT_UPPER_LIMIT);

        m_pid = new PIDController(0.2,0.0, 0.0);

        if (RobotBase.isSimulation()) {
            ElevatorSim sim = new ElevatorSim(
                    ElevatorSimConstants.K_ELEVATOR_GEARBOX,
                    ElevatorSimConstants.K_ELEVATOR_GEARING,
                    ElevatorSimConstants.K_CARRIAGE_MASS,
                    ElevatorSimConstants.K_ELEVATOR_DRUM_RADIUS,
                    ElevatorSimConstants.K_MIN_ELEVATOR_HEIGHT,
                    ElevatorSimConstants.K_MAX_ELEVATOR_HEIGHT, true);

            m_elevatorSim = new ElevatorSimWrapper(sim,
                    new RevMotorControllerSimWrapper(m_liftMotor),
                    RevEncoderSimWrapper.create(m_liftMotor));
            m_elevatorSim.setLowerLimitSwitch(new BaseDigitalInputWrapper(new DIOSim(m_lowerLimitSwitch)::setValue));
            m_elevatorSim.setUpperLimitSwitch(new BaseDigitalInputWrapper(new DIOSim(m_upperLimitSwitch)::setValue));

            // Create a Mechanism2d for visualization
            m_mech = new Mechanism2d(10, 10);
            m_root = m_mech.getRoot("root", 2.5, 0.25);
            m_elevator = m_root.append(new MechanismLigament2d("elevator", ElevatorSimConstants.K_MIN_ELEVATOR_HEIGHT, 90, 6, new Color8Bit(Color.kFirstRed)));
            m_elevator.append(new MechanismLigament2d("Manipulator", 1.5, 270, 6, new Color8Bit(Color.kAliceBlue)));

            SmartDashboard.putData("Elevatorsim", m_mech);
        }
    }

    @Override
    public void close() {
        m_liftMotor.close();
        m_lowerLimitSwitch.close();
        m_upperLimitSwitch.close();
    }

    @Override
    public void periodic() {
        m_elevator.setLength(getHeight());
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.update();
    }

    public void goToPosition(double position) {
        double output = m_pid.calculate(getHeight(), position);
        setSpeed(output);
    }

    public boolean isAtLowerLimit() {
        return m_lowerLimitSwitch.get();
    }

    public boolean isAtUpperLimit() {
        return m_upperLimitSwitch.get();
    }

    public void stop() {
        m_liftMotor.stopMotor();
    }

    // 1 -> -1
    public void setSpeed(double speed) {
        m_liftMotor.set(speed);
    }

    // get how high the elevator is in the air
    public double getHeight() {
        return m_liftEncoder.getPosition() * 360 * ElevatorSimConstants.K_ELEVATOR_DRUM_RADIUS;
    }
}
