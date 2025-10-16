package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
    
    public static final class ArmConstants {
        // CAN ID - 30 - one in the back of the robot for now
        public static final int ARM_MOTOR_CAN_ID = 30;
        
        // Physical properties
        public static final double GEAR_RATIO = 125.0;
        public static final double ARM_LENGTH_INCHES = 19.0;
        
        // Positions (degrees, 0 = straight down)
        public static final double REST_POSITION = 0.0;
        public static final double SCORE_45_POSITION = 45.0 + 90.0;
        public static final double SCORE_60_POSITION = 60.0 + 90.0;
        public static final double VERTICAL_POSITION = 180.0;
        
        // PID Constants (TUNE THESE VALUES)
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.01;
        public static final double kFF = 0.0;
        
        // Gravity feedforward (TUNE THIS VALUE)
        public static final double kG = 0.05;
        
        // Motion profiling
        public static final double MAX_VELOCITY = 100.0; // deg/s
        public static final double MAX_ACCELERATION = 150.0; // deg/s²
        
        // Soft limits (degrees)
        public static final double LOWER_LIMIT = -5.0;
        public static final double UPPER_LIMIT = 185.0;
        
        // Current limits
        public static final int CURRENT_LIMIT = 30; // Amps
        
        // Tolerances
        public static final double POSITION_TOLERANCE = 2.0; // degrees
        
        // Output limits
        public static final double MAX_OUTPUT = 0.5;
        public static final double MIN_OUTPUT = -0.5;
    }
    
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
}