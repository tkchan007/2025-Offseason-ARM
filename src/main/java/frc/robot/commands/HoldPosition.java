package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command to hold the current position
 * This is used as the default command for the arm
 */
public class HoldPosition extends Command {
    private final ArmSubsystem armSubsystem;
    
    public HoldPosition(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        // Set target to current position
        armSubsystem.setTargetAngle(armSubsystem.getArmAngle());
    }
    
    @Override
    public void execute() {
        armSubsystem.updateMotionProfile();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}