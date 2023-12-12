package frc.robot;

public class Constants {
    public static class DrivetrainConstants {
        public static final int frontLeftID = 1;
        public static final int frontRightID = 3;
        public static final int backLeftID = 2;
        public static final int backRightID = 4;
        
        public static final double maxDriveSpeed = 0.5;
        public static final double maxTurnSpeed = 0.4;

        public static final double slewRate = 0.3;

        public static class PIDConstants {
            // public static final double rotP = 0.00000001;
            // public static final double rotI = 0;
            // public static final double rotD = 0.00000000001;
            public static final double rotP = 0.015;
            public static final double rotI = 0;
            public static final double rotD = 0.001;
            public static final double rotSetpoint = 0;
            public static final double rotTolerance = 1;
            public static final double rotClamp = 0.3;
        }
    }

    public static class ArmConstants {
        public static final int extensionID = 9;
        public static final int inclineID1 = 0;
        public static final int inclineID2 = 6;
        public static final int clawID = 7;

        public static final double incline2DefaultPos = 25000;

        public static final double armMaxAngle = 43;
        public static final double armMinAngleAtOut = 30;
    }

    public static class DriverConstants {
        public static final double deadband = 0.08;
        public static final double limitDeadband = 0.01;
        public static final int port = 0;

        public static final int forwardAxis = 1;
        public static final int sideAxis = 2;
        public static final int rotAxis = 0;
        public static final int scalingAxis = 5;
    }
}
