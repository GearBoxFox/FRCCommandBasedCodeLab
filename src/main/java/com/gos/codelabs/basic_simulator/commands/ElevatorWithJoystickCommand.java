package com.gos.codelabs.basic_simulator.commands;

import com.gos.codelabs.basic_simulator.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ElevatorWithJoystickCommand extends CommandBase {
    private final CommandXboxController m_operatorJoystick;
    private final ElevatorSubsystem m_lift;


    public ElevatorWithJoystickCommand(ElevatorSubsystem lift, CommandXboxController operatorJoystick) {
        m_lift = lift;
        m_operatorJoystick = operatorJoystick;

        addRequirements(lift);
    }

    @Override
    public void execute() {
        m_lift.setSpeed(-m_operatorJoystick.getLeftY() * 0.5);
    }
}
