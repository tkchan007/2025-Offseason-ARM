package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;
    
    // Constants
    private static final int ARM_MOTOR_CAN_ID = Constants.ArmConstants.ARM_MOTOR_CAN_ID; 
    private static final double GEAR_RATIO = Constants.ArmConstants.GEAR_RATIO;
    private static final double ARM_LENGTH_INCHES = Constants.ArmConstants.ARM_LENGTH_INCHES;
    
    // Arm positions in degrees (0 = straight down)
    public static final double REST_POSITION = Constants.ArmConstants.REST_POSITION;      // Straight down
    public static final double SCORE_45_POSITION = Constants.ArmConstants.SCORE_45_POSITION;  // 45° above rest
    public static final double SCORE_60_POSITION = Constants.ArmConstants.SCORE_60_POSITION;  // 60° above rest
    public static final double VERTICAL_POSITION = Constants.ArmConstants.VERTICAL_POSITION; // Straight up
    
    // Default PID Constants (loaded from code)
    private static final double DEFAULT_KP = 0.1;
    private static final double DEFAULT_KI = 0.0;
    private static final double DEFAULT_KD = 0.01;
    private static final double DEFAULT_KG = 0.05;
    
    // Default motion profiling constraints
    private static final double DEFAULT_MAX_VELOCITY = 100.0; // degrees per second
    private static final double DEFAULT_MAX_ACCELERATION = 150.0; // degrees per second squared
    
    // Soft limits (in degrees)
    private static final double LOWER_LIMIT_DEGREES = -5.0;
    private static final double UPPER_LIMIT_DEGREES = 185.0;
    
    // Motion profile
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State setpointState;
    
    // Calibration mode flag
    private boolean calibrationMode = false;
    
    // Store last known values to detect changes
    private double lastKP = DEFAULT_KP;
    private double lastKI = DEFAULT_KI;
    private double lastKD = DEFAULT_KD;
    private double lastKG = DEFAULT_KG;
    private double lastMaxVel = DEFAULT_MAX_VELOCITY;
    private double lastMaxAccel = DEFAULT_MAX_ACCELERATION;
    
    public ArmSubsystem() {
        // Initialize motor
        armMotor = new SparkMax(ARM_MOTOR_CAN_ID, MotorType.kBrushless);
        
        // Get encoder and closed loop controller
        encoder = armMotor.getEncoder();
        closedLoopController = armMotor.getClosedLoopController();
        
        // Create configuration object
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Configure encoder conversion factors
        config.encoder
            .positionConversionFactor(360.0 / GEAR_RATIO)  // Convert to degrees
            .velocityConversionFactor(360.0 / GEAR_RATIO / 60.0);  // Convert to deg/s
        
        // Configure PID values
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(DEFAULT_KP)
            .i(DEFAULT_KI)
            .d(DEFAULT_KD)
            .outputRange(-0.5, 0.5);  // Limit speed for safety
        
        // Configure soft limits
        config.softLimit
            .forwardSoftLimit((float)UPPER_LIMIT_DEGREES)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit((float)LOWER_LIMIT_DEGREES)
            .reverseSoftLimitEnabled(true);
        
        // Configure current limits
        config.smartCurrentLimit(30);  // Amps
        
        // Enable voltage compensation
        config.voltageCompensation(12.0);
        
        // Set idle mode to brake
        config.idleMode(IdleMode.kBrake);
        
        // Apply configuration and persist to flash
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Initialize motion profile
        constraints = new TrapezoidProfile.Constraints(DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCELERATION);
        profile = new TrapezoidProfile(constraints);
        goalState = new TrapezoidProfile.State(REST_POSITION, 0.0);
        setpointState = new TrapezoidProfile.State(getArmAngle(), 0.0);
        
        // Initialize dashboard values
        initializeDashboard();
        
        // Zero encoder
        resetEncoder();
    }
    
    /**
     * Initialize all dashboard values for calibration and live operation
     */
    private void initializeDashboard() {
        // === CALIBRATION VALUES (CAL) ===
        SmartDashboard.putNumber("CAL kP", DEFAULT_KP);
        SmartDashboard.putNumber("CAL kI", DEFAULT_KI);
        SmartDashboard.putNumber("CAL kD", DEFAULT_KD);
        SmartDashboard.putNumber("CAL kG", DEFAULT_KG);
        SmartDashboard.putNumber("CAL Max Velocity", DEFAULT_MAX_VELOCITY);
        SmartDashboard.putNumber("CAL Max Accel", DEFAULT_MAX_ACCELERATION);
        SmartDashboard.putNumber("CAL Target Angle", 45.0);
        SmartDashboard.putString("CAL Status", "Ready");
        SmartDashboard.putBoolean("CAL Mode Active", false);
        
        // === LIVE VALUES (LIVE) - Read-only indicators ===
        SmartDashboard.putNumber("LIVE kP", DEFAULT_KP);
        SmartDashboard.putNumber("LIVE kI", DEFAULT_KI);
        SmartDashboard.putNumber("LIVE kD", DEFAULT_KD);
        SmartDashboard.putNumber("LIVE kG", DEFAULT_KG);
        SmartDashboard.putNumber("LIVE Max Velocity", DEFAULT_MAX_VELOCITY);
        SmartDashboard.putNumber("LIVE Max Accel", DEFAULT_MAX_ACCELERATION);
        
        // === TELEMETRY ===
        SmartDashboard.putNumber("Arm Angle", 0.0);
        SmartDashboard.putNumber("Arm Velocity", 0.0);
        SmartDashboard.putNumber("Arm Current", 0.0);
        SmartDashboard.putNumber("Arm Goal", 0.0);
        SmartDashboard.putNumber("Arm Error", 0.0);
        SmartDashboard.putString("Arm Status", "Initialized");
    }
    
    @Override
    public void periodic() {
        // Update telemetry
        SmartDashboard.putNumber("Arm Angle", getArmAngle());
        SmartDashboard.putNumber("Arm Velocity", getArmVelocity());
        SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm Goal", goalState.position);
        SmartDashboard.putNumber("Arm Error", goalState.position - getArmAngle());
        SmartDashboard.putNumber("Arm Motor Temp", armMotor.getMotorTemperature());
        
        // Check which mode we're in
        calibrationMode = SmartDashboard.getBoolean("CAL Mode Active", false);
        
        // Update parameters based on active mode
        if (calibrationMode) {
            updateFromCalibrationValues();
            SmartDashboard.putString("Arm Status", "CALIBRATION MODE");
        } else {
            updateFromLiveValues();
            SmartDashboard.putString("Arm Status", "Normal Operation");
        }
    }
    
    /**
     * Update motor controller from CALIBRATION values (CAL)
     * Used during tuning and testing
     */
    private void updateFromCalibrationValues() {
        double kP = SmartDashboard.getNumber("CAL kP", DEFAULT_KP);
        double kI = SmartDashboard.getNumber("CAL kI", DEFAULT_KI);
        double kD = SmartDashboard.getNumber("CAL kD", DEFAULT_KD);
        double maxVel = SmartDashboard.getNumber("CAL Max Velocity", DEFAULT_MAX_VELOCITY);
        double maxAccel = SmartDashboard.getNumber("CAL Max Accel", DEFAULT_MAX_ACCELERATION);
        
        // Update PID if changed
        if (kP != lastKP || kI != lastKI || kD != lastKD) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop
                .p(kP, ClosedLoopSlot.kSlot0)
                .i(kI, ClosedLoopSlot.kSlot0)
                .d(kD, ClosedLoopSlot.kSlot0);
            
            armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            
            lastKP = kP;
            lastKI = kI;
            lastKD = kD;
            System.out.println("PID Updated: P=" + kP + " I=" + kI + " D=" + kD);
        }
        
        // Update motion constraints if changed
        if (maxVel != lastMaxVel || maxAccel != lastMaxAccel) {
            constraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
            profile = new TrapezoidProfile(constraints);
            lastMaxVel = maxVel;
            lastMaxAccel = maxAccel;
            System.out.println("Motion Profile Updated: Vel=" + maxVel + " Accel=" + maxAccel);
        }
        
        // kG is read directly in calculateFeedforward()
    }
    
    /**
     * Update motor controller from LIVE values
     * Used during normal competition operation
     */
    private void updateFromLiveValues() {
        double kP = SmartDashboard.getNumber("LIVE kP", DEFAULT_KP);
        double kI = SmartDashboard.getNumber("LIVE kI", DEFAULT_KI);
        double kD = SmartDashboard.getNumber("LIVE kD", DEFAULT_KD);
        double maxVel = SmartDashboard.getNumber("LIVE Max Velocity", DEFAULT_MAX_VELOCITY);
        double maxAccel = SmartDashboard.getNumber("LIVE Max Accel", DEFAULT_MAX_ACCELERATION);
        
        // Update PID if changed
        if (kP != lastKP || kI != lastKI || kD != lastKD) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop
                .p(kP, ClosedLoopSlot.kSlot0)
                .i(kI, ClosedLoopSlot.kSlot0)
                .d(kD, ClosedLoopSlot.kSlot0);
            
            armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            
            lastKP = kP;
            lastKI = kI;
            lastKD = kD;
        }
        
        // Update motion constraints if changed
        if (maxVel != lastMaxVel || maxAccel != lastMaxAccel) {
            constraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
            profile = new TrapezoidProfile(constraints);
            lastMaxVel = maxVel;
            lastMaxAccel = maxAccel;
        }
    }
    
    /**
     * Set the target position for the arm using motion profiling
     */
    public void setTargetAngle(double targetAngle) {
        goalState = new TrapezoidProfile.State(targetAngle, 0.0);
    }
    
    /**
     * Update the arm position using motion profiling
     */
    public void updateMotionProfile() {
        // In WPILib 2025, calculate() takes (time, currentState, goalState) parameters
        setpointState = profile.calculate(0.02, setpointState, goalState);
        
        double feedforward = calculateFeedforward(setpointState.position);
        
        closedLoopController.setReference(
            setpointState.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward
        );
    }
    
    /**
     * Calculate feedforward to compensate for gravity
     */
    private double calculateFeedforward(double angle) {
        // Get kG from appropriate source based on mode
        double kG;
        if (calibrationMode) {
            kG = SmartDashboard.getNumber("CAL kG", DEFAULT_KG);
        } else {
            kG = SmartDashboard.getNumber("LIVE kG", DEFAULT_KG);
        }
        
        // Shifted by 90° so maximum feedforward is when arm is horizontal
        return kG * Math.cos(Math.toRadians(angle - 90));
    }
    
    /**
     * Get current arm angle in degrees
     */
    public double getArmAngle() {
        return encoder.getPosition();
    }
    
    /**
     * Get current arm velocity in degrees per second
     */
    public double getArmVelocity() {
        return encoder.getVelocity();
    }
    
    /**
     * Check if arm is at target position
     */
    public boolean atTargetPosition(double tolerance) {
        return Math.abs(getArmAngle() - goalState.position) < tolerance;
    }
    
    /**
     * Reset encoder to zero
     */
    public void resetEncoder() {
        encoder.setPosition(REST_POSITION);
        System.out.println("Encoder reset to " + REST_POSITION);
    }
    
    /**
     * Stop the arm motor
     */
    public void stop() {
        armMotor.stopMotor();
    }
    
    /**
     * Manual control for calibration
     */
    public void setManualSpeed(double speed) {
        armMotor.set(speed);
    }
    
    /**
     * Enable calibration mode
     */
    public void enableCalibrationMode() {
        calibrationMode = true;
        SmartDashboard.putBoolean("CAL Mode Active", true);
        System.out.println(">>> CALIBRATION MODE ENABLED <<<");
    }
    
    /**
     * Disable calibration mode
     */
    public void disableCalibrationMode() {
        calibrationMode = false;
        SmartDashboard.putBoolean("CAL Mode Active", false);
        System.out.println(">>> CALIBRATION MODE DISABLED <<<");
    }
    
    /**
     * Check if in calibration mode
     */
    public boolean isCalibrationMode() {
        return calibrationMode;
    }
    
    // ============================================================================
    // COMMAND FACTORY METHODS
    // ============================================================================
    // These methods return Command objects that can be bound to buttons.
    // They follow the pattern of "methods that return behaviors" - similar to
    // how you might have getSpeed() or setPosition() methods, these are
    // "movement commands" modeled as methods on the subsystem.
    //
    // Benefits of this approach:
    // - Type armSubsystem. in your IDE and see all available operations
    // - Subsystem encapsulates its own capabilities
    // - Cleaner RobotContainer code
    // - Easier for students to discover what the arm can do
    // ============================================================================
    
    /**
     * Command to move arm to rest position (0°, straight down)
     * 
     * This is modeled as a method that returns a Command object.
     * Think of it like: "Give me the command to move to rest"
     * 
     * @return Command that moves arm to rest position
     */
    public Command moveToRestCommand() {
        return this.run(() -> {
            setTargetAngle(REST_POSITION);
            updateMotionProfile();
        }).until(() -> atTargetPosition(2.0))
          .withName("MoveToRest");
    }
    
    /**
     * Command to move arm to 45° position
     * 
     * @return Command that moves arm to 45° above rest
     */
    public Command moveTo45Command() {
        return this.run(() -> {
            setTargetAngle(SCORE_45_POSITION);
            updateMotionProfile();
        }).until(() -> atTargetPosition(2.0))
          .withName("MoveTo45");
    }
    
    /**
     * Command to move arm to 60° position
     * 
     * @return Command that moves arm to 60° above rest
     */
    public Command moveTo60Command() {
        return this.run(() -> {
            setTargetAngle(SCORE_60_POSITION);
            updateMotionProfile();
        }).until(() -> atTargetPosition(2.0))
          .withName("MoveTo60");
    }
    
    /**
     * Command to move arm to vertical position (180°, straight up)
     * 
     * @return Command that moves arm to vertical position
     */
    public Command moveToVerticalCommand() {
        return this.run(() -> {
            setTargetAngle(VERTICAL_POSITION);
            updateMotionProfile();
        }).until(() -> atTargetPosition(2.0))
          .withName("MoveToVertical");
    }
    
    /**
     * Command to hold the current arm position
     * 
     * This command continuously updates the motion profile to maintain
     * the current position. It runs indefinitely until interrupted by
     * another command.
     * 
     * This is typically used as the default command for the arm subsystem.
     * 
     * @return Command that holds current position
     */
    public Command holdPositionCommand() {
        return this.run(() -> {
            // On first execution, set target to current position
            if (Math.abs(goalState.position - getArmAngle()) > 5.0) {
                setTargetAngle(getArmAngle());
            }
            updateMotionProfile();
        }).withName("HoldPosition");
    }
    
    /**
     * Generic command to move arm to any angle
     * 
     * This provides flexibility for custom positions beyond the presets.
     * 
     * Example usage: armSubsystem.moveToAngleCommand(75.0)
     * 
     * @param targetAngle Target angle in degrees
     * @return Command that moves arm to specified angle
     */
    public Command moveToAngleCommand(double targetAngle) {
        return this.run(() -> {
            setTargetAngle(targetAngle);
            updateMotionProfile();
        }).until(() -> atTargetPosition(2.0))
          .withName("MoveToAngle(" + targetAngle + "°)");
    }
}
