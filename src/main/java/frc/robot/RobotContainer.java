// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;

// Import calibration commands explicitly
import frc.robot.commands.calibration.CAL_TestGravityFeedforward;
import frc.robot.commands.calibration.CAL_ManualArmControl;
import frc.robot.commands.calibration.CAL_TunePID;
import frc.robot.commands.calibration.CAL_TuneMotionProfile;
import frc.robot.commands.calibration.CAL_DisturbanceTest;
import frc.robot.commands.calibration.CAL_SaveCalibration;
import frc.robot.commands.calibration.CAL_LoadLiveValues;

/**
 * This class defines the structure of the robot including subsystems, 
 * commands, and button mappings for both normal operation and calibration.
 * 
 * Notice how the commands are accessed:
 * - Normal operation: armSubsystem.moveToRestCommand() - like calling a method
 * - Calibration: new CAL_TunePID(armSubsystem) - explicit command objects
 */
public class RobotContainer {
    // Subsystems
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    
    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public RobotContainer() {
        configureBindings();
        
        // Set default command to hold position
        // This demonstrates the command factory method pattern - we call a method
        // on the subsystem that returns a Command object
        armSubsystem.setDefaultCommand(armSubsystem.holdPositionCommand());
    }
    
    /**
     * Configure button bindings for normal operation and calibration
     */
    private void configureBindings() {
        // ============================================================
        // NORMAL OPERATION BINDINGS (Operator Controller)
        // ============================================================
        // These use the command factory methods on ArmSubsystem.
        // Notice the pattern: subsystem.commandNameCommand()
        //
        // This is like calling methods on an object:
        //   armSubsystem.moveToRestCommand() 
        //   returns a Command that moves the arm to rest
        // ============================================================
        
        // A button - Move to rest position (straight down)
        operatorController.a()
            .onTrue(armSubsystem.moveToRestCommand());
        
        // B button - Move to 45° position
        operatorController.b()
            .onTrue(armSubsystem.moveTo45Command());
        
        // X button - Move to 60° position
        operatorController.x()
            .onTrue(armSubsystem.moveTo60Command());
        
        // Y button - Move to vertical position (180°)
        operatorController.y()
            .onTrue(armSubsystem.moveToVerticalCommand());
        
        // ============================================================
        // CALIBRATION MODE BINDINGS (Driver Controller)
        // ============================================================
        // These use explicit Command classes from the calibration package.
        // Calibration commands are separate because they're diagnostic tools,
        // not regular robot operations.
        // ============================================================
        
        // START + BACK together - Toggle calibration mode
        new Trigger(() -> driverController.start().getAsBoolean() && 
                         driverController.back().getAsBoolean())
            .onTrue(new InstantCommand(() -> {
                if (armSubsystem.isCalibrationMode()) {
                    armSubsystem.disableCalibrationMode();
                } else {
                    armSubsystem.enableCalibrationMode();
                }
            }));
        
        // CAL: Left trigger - Manual control DOWN (while held)
        driverController.leftTrigger(0.1)
            .whileTrue(new CAL_ManualArmControl(armSubsystem, -0.15));
        
        // CAL: Right trigger - Manual control UP (while held)
        driverController.rightTrigger(0.1)
            .whileTrue(new CAL_ManualArmControl(armSubsystem, 0.15));
        
        // CAL: A button - Test Gravity Feedforward (hold while adjusting kG)
        driverController.a()
            .whileTrue(new CAL_TestGravityFeedforward(armSubsystem));
        
        // CAL: B button - PID Tuning to 45° target
        driverController.b()
            .whileTrue(new CAL_TunePID(armSubsystem));
        
        // CAL: X button - Motion Profile Test (0° to 180°)
        driverController.x()
            .onTrue(new CAL_TuneMotionProfile(armSubsystem, 0.0, 180.0));
        
        // CAL: Y button - Motion Profile Test (180° to 0°)
        driverController.y()
            .onTrue(new CAL_TuneMotionProfile(armSubsystem, 180.0, 0.0));
        
        // CAL: Left bumper - Disturbance test at 90°
        driverController.leftBumper()
            .whileTrue(new CAL_DisturbanceTest(armSubsystem, 90.0));
        
        // CAL: Right bumper - Save calibration values to LIVE
        driverController.rightBumper()
            .onTrue(new CAL_SaveCalibration(armSubsystem));
        
        // CAL: POV Up - Load LIVE values to CAL for further tuning
        driverController.povUp()
            .onTrue(new CAL_LoadLiveValues(armSubsystem));
        
        // CAL: POV Down - Reset encoder (arm must be at rest position)
        driverController.povDown()
            .onTrue(new InstantCommand(() -> armSubsystem.resetEncoder())
                .ignoringDisable(true));
        
        // UTILITY: Back button (operator) - Emergency reset encoder
        operatorController.back()
            .onTrue(new InstantCommand(() -> armSubsystem.resetEncoder())
                .ignoringDisable(true));
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Example autonomous sequence using command factory methods
        // Notice how readable this is - it's like writing instructions:
        // "Move to vertical, then to 45, then to rest"
        return armSubsystem.moveToVerticalCommand()
            .andThen(armSubsystem.moveTo45Command())
            .andThen(armSubsystem.moveToRestCommand());
    }
}
