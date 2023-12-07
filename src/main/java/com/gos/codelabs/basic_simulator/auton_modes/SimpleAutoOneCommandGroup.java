package com.gos.codelabs.basic_simulator.auton_modes;


import com.gos.codelabs.basic_simulator.commands.ElevatorToPositionCommand;
import com.gos.codelabs.basic_simulator.subsystems.ChassisSubsystem;
import com.gos.codelabs.basic_simulator.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimpleAutoOneCommandGroup extends SequentialCommandGroup {
    public SimpleAutoOneCommandGroup(ChassisSubsystem m_drive, ElevatorSubsystem m_lift) {
        addCommands(new DriveForTimeCommand(m_drive, 0.5, 5));
        addCommands(new TurnForTimeCommand(m_drive, 0.5, 3));
        addCommands(new ElevatorToPositionCommand(m_lift, ElevatorSubsystem.Positions.HIGH));
    }
}