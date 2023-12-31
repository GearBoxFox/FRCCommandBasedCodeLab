package com.gos.codelabs.basic_simulator.subsystems;

import com.gos.codelabs.basic_simulator.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PunchSubsystem extends SubsystemBase implements AutoCloseable {

    private final Solenoid m_punchSolenoid;

    public PunchSubsystem() {
        m_punchSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_PUNCH);
    }


    @Override
    public void close() {
        m_punchSolenoid.close();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Solenoid Extended?", isExtended());
    }

    public boolean isExtended() {
        // TODO implement
        return m_punchSolenoid.get();
    }

    public void extend() {
        // TODO implement
        m_punchSolenoid.set(true);
    }

    public void retract() {
        // TODO implement
        m_punchSolenoid.set(false);
    }
}
