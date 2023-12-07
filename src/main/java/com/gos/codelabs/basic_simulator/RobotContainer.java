// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gos.codelabs.basic_simulator;

import com.gos.codelabs.basic_simulator.auton_modes.AutonFactory;
import com.gos.codelabs.basic_simulator.auton_modes.DriveForTimeCommand;
import com.gos.codelabs.basic_simulator.auton_modes.SimpleAutoOneCommandGroup;
import com.gos.codelabs.basic_simulator.auton_modes.TurnForTimeCommand;
import com.gos.codelabs.basic_simulator.commands.ElevatorToPositionCommand;
import com.gos.codelabs.basic_simulator.commands.ElevatorWithJoystickCommand;
import com.gos.codelabs.basic_simulator.commands.MovePunchCommand;
import com.gos.codelabs.basic_simulator.subsystems.ChassisSubsystem;
import com.gos.codelabs.basic_simulator.subsystems.ElevatorSubsystem;
import com.gos.codelabs.basic_simulator.subsystems.PunchSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("PMD.SingularField")
public class RobotContainer implements AutoCloseable {
    // Subsystems
    private final ChassisSubsystem m_chassisSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final PunchSubsystem m_punchSubsystem;

    // Joysticks
    private final CommandXboxController m_driverJoystick;
    private final CommandXboxController m_operatorJoystick;

    private final SendableChooser<Command> m_autonFactory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_chassisSubsystem = new ChassisSubsystem();
        m_elevatorSubsystem = new ElevatorSubsystem();
        m_punchSubsystem = new PunchSubsystem();

        m_driverJoystick = new CommandXboxController(0);
        m_operatorJoystick = new CommandXboxController(1);

        new CommandTester(this);

        m_autonFactory = new SendableChooser<>();
        m_autonFactory.setDefaultOption("Drive for 5 seconds", new DriveForTimeCommand(m_chassisSubsystem, 0.5, 5));
        m_autonFactory.addOption("Turn for 5 seconds", new TurnForTimeCommand(m_chassisSubsystem, 0.5, 5));
        m_autonFactory.addOption("Simple auto", new SimpleAutoOneCommandGroup(m_chassisSubsystem, m_elevatorSubsystem));

        SmartDashboard.putData("Auton Chooser", m_autonFactory);

        // Configure the button bindings
        configureButtonBindings();
    }

    @Override
    public void close() {
        m_chassisSubsystem.close();
        m_elevatorSubsystem.close();
        m_punchSubsystem.close();
    }

    private void configureButtonBindings() {
//        m_elevatorSubsystem.setDefaultCommand(new ElevatorWithJoystickCommand(m_elevatorSubsystem, m_driverJoystick));

        m_driverJoystick.a().whileTrue(new MovePunchCommand(m_punchSubsystem, true));
        m_driverJoystick.b().onTrue(new MovePunchCommand(m_punchSubsystem, false));

        m_driverJoystick.rightTrigger().whileTrue(new ElevatorToPositionCommand(m_elevatorSubsystem, ElevatorSubsystem.Positions.MID));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autonFactory.getSelected();
    }

    public ElevatorSubsystem getElevator() {
        return m_elevatorSubsystem;
    }

    public ChassisSubsystem getChassis() {
        return m_chassisSubsystem;
    }

    public PunchSubsystem getPunch() {
        return m_punchSubsystem;
    }
}
