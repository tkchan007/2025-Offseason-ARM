package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CALIBRATION COMMAND: Test gravity feedforward at current arm position
 * This command reads kG from SmartDashboard and applies calculated feedforward
 * to hold the arm at its current position.
 * 
 * Usage: Hold button while adjusting "CAL kG" value on dashboard
 */
public class CAL_TestGravityFeedforward extends Command {
    private final ArmSubsystem armSubsystem;
    
    public CAL_TestGravityFeedforward(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println(">>> CALIBRATION MODE: Testing Gravity Feedforward <<<");
        System.out.println("Adjust 'CAL kG' value on dashboard");
        System.out.println("Arm should hold position without drifting");
    }
    
    @Override
    public void execute() {
        // Read test kG value from dashboard
        double testKG = SmartDashboard.getNumber("CAL kG", 0.0);
        double currentAngle = armSubsystem.getArmAngle();
        
        // Calculate feedforward: kG * cos(angle - 90°)
        // Shifted by 90° so maximum feedforward is when arm is horizontal
        double feedforward = testKG * Math.cos(Math.toRadians(currentAngle - 90));
        
        // Apply directly to motor
        armSubsystem.setManualSpeed(feedforward);
        
        // Update dashboard
        SmartDashboard.putNumber("CAL Applied FF", feedforward);
        SmartDashboard.putNumber("CAL Current Angle", currentAngle);
        SmartDashboard.putString("CAL Status", "TESTING - Observe arm drift");
    }
    
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
        SmartDashboard.putString("CAL Status", "TEST ENDED");
        System.out.println(">>> Gravity FF Test Ended <<<");
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until button released
    }
}
