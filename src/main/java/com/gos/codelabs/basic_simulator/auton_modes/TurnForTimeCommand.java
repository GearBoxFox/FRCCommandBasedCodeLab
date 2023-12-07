package com.gos.codelabs.basic_simulator.auton_modes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.codelabs.basic_simulator.subsystems.ChassisSubsystem;


public class TurnForTimeCommand extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;
    double time, turnSpeed;
    private Timer m_timer;

    public TurnForTimeCommand(ChassisSubsystem chassisSubsystem, double time, double turnSpeed) {
        this.chassisSubsystem = chassisSubsystem;
        this.time = time;
        this.turnSpeed = turnSpeed;
        m_timer = new Timer();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.chassisSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        chassisSubsystem.arcadeDrive(0.0, turnSpeed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return m_timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        chassisSubsystem.stop();
    }
}
