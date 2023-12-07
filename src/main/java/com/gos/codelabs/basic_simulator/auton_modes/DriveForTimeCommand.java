package com.gos.codelabs.basic_simulator.auton_modes;

import com.gos.codelabs.basic_simulator.subsystems.ChassisSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveForTimeCommand extends CommandBase {

    ChassisSubsystem m_drive;
    double speed, time;
    Timer m_timer;

    public DriveForTimeCommand(ChassisSubsystem chassis, double speed, double time) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_timer = new Timer();
        m_drive = chassis;
        this.speed = speed;
        this.time = time;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_drive.arcadeDrive(speed, 0.0);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
