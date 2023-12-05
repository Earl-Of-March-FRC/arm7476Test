package frc.robot;

public class Constants {
    public static class DrivetrainConstants {
        public static final int frontLeftID = 1;
        public static final int frontRightID = 3;
        public static final int backLeftID = 2;
        public static final int backRightID = 4;
        
        public static final double maxDriveSpeed = 0.5;
        public static final double maxTurnSpeed = 0.4;

        public static final double slewRate = 0.4;

        public static class PIDConstants {
            public static final double rotP = 0.0000001;
            public static final double rotI = 0;
            public static final double rotD = 0.0000000001;
            public static final double rotTolerance = 5;
            public static final double rotClamp = 0.3;
        }
    }
    public static class DriverConstants {
        public static final double deadband = 0.08;
        public static final double limitDeadband = 0.01;
        public static final int port = 0;

        public static final int forwardAxis = 1;
        public static final int sideAxis = 0;
        public static final int rotAxis = 4;
        public static final int scalingAxis = 5;
    }
}
