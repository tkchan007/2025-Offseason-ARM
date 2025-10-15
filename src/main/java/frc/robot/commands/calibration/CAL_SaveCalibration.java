package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CALIBRATION COMMAND: Save calibration values
 * Copies current CAL values to LIVE values
 */
public class CAL_SaveCalibration extends Command {
    private final ArmSubsystem armSubsystem;
    
    public CAL_SaveCalibration(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        // Don't require subsystem - this is just a dashboard operation
    }
    
    @Override
    public void initialize() {
        // Copy calibration values to live values
        double kP = SmartDashboard.getNumber("CAL kP", 0.0);
        double kI = SmartDashboard.getNumber("CAL kI", 0.0);
        double kD = SmartDashboard.getNumber("CAL kD", 0.0);
        double kG = SmartDashboard.getNumber("CAL kG", 0.0);
        double maxVel = SmartDashboard.getNumber("CAL Max Velocity", 100.0);
        double maxAccel = SmartDashboard.getNumber("CAL Max Accel", 150.0);
        
        SmartDashboard.putNumber("LIVE kP", kP);
        SmartDashboard.putNumber("LIVE kI", kI);
        SmartDashboard.putNumber("LIVE kD", kD);
        SmartDashboard.putNumber("LIVE kG", kG);
        SmartDashboard.putNumber("LIVE Max Velocity", maxVel);
        SmartDashboard.putNumber("LIVE Max Accel", maxAccel);
        
        System.out.println(">>> CALIBRATION VALUES SAVED TO LIVE <<<");
        System.out.println("kP: " + kP);
        System.out.println("kI: " + kI);
        System.out.println("kD: " + kD);
        System.out.println("kG: " + kG);
        System.out.println("Max Velocity: " + maxVel);
        System.out.println("Max Acceleration: " + maxAccel);
        System.out.println(">>> UPDATE CONSTANTS IN CODE <<<");
        
        SmartDashboard.putString("CAL Status", "✓ VALUES SAVED TO LIVE - Update code!");
    }
    
    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}