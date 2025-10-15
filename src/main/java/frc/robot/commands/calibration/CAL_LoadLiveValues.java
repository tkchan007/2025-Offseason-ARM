package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CALIBRATION COMMAND: Load live values to calibration
 * Copies current LIVE values to CAL values for further tuning
 */
public class CAL_LoadLiveValues extends Command {
    private final ArmSubsystem armSubsystem;
    
    public CAL_LoadLiveValues(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }
    
    @Override
    public void initialize() {
        // Copy live values to calibration values
        double kP = SmartDashboard.getNumber("LIVE kP", 0.0);
        double kI = SmartDashboard.getNumber("LIVE kI", 0.0);
        double kD = SmartDashboard.getNumber("LIVE kD", 0.0);
        double kG = SmartDashboard.getNumber("LIVE kG", 0.0);
        double maxVel = SmartDashboard.getNumber("LIVE Max Velocity", 100.0);
        double maxAccel = SmartDashboard.getNumber("LIVE Max Accel", 150.0);
        
        SmartDashboard.putNumber("CAL kP", kP);
        SmartDashboard.putNumber("CAL kI", kI);
        SmartDashboard.putNumber("CAL kD", kD);
        SmartDashboard.putNumber("CAL kG", kG);
        SmartDashboard.putNumber("CAL Max Velocity", maxVel);
        SmartDashboard.putNumber("CAL Max Accel", maxAccel);
        
        System.out.println(">>> LIVE VALUES LOADED TO CALIBRATION <<<");
        SmartDashboard.putString("CAL Status", "Live values loaded - Ready to tune");
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}