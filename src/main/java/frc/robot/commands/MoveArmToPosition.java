package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command to move arm to a specific position using motion profiling
 */
public class MoveArmToPosition extends Command {
    private final ArmSubsystem armSubsystem;
    private final double targetAngle;
    private static final double POSITION_TOLERANCE = 2.0; // degrees
    
    public MoveArmToPosition(ArmSubsystem armSubsystem, double targetAngle) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        armSubsystem.setTargetAngle(targetAngle);
    }
    
    @Override
    public void execute() {
        armSubsystem.updateMotionProfile();
    }
    
    @Override
    public boolean isFinished() {
        return armSubsystem.atTargetPosition(POSITION_TOLERANCE);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Keep holding position even when command ends
        // The PID controller will maintain position
    }
}

