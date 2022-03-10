package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Shooter {

    WPI_TalonFX masterShooterMotor = new WPI_TalonFX(Constants.masterShooterMotorIndex);
    WPI_TalonFX slaveShooterMotor = new WPI_TalonFX(Constants.slaveShooterMotorIndex);
    CANSparkMax hoodMotor = new CANSparkMax(Constants.hoodMotorIndex, MotorType.kBrushless);
    PWMSparkMax pwmSparkMax = new PWMSparkMax(0);

    boolean shooting = false;
    boolean dumpShot = false;

    final double saturationVoltage = 12.5;

    public void resetHoodEncoders() {
        hoodMotor.getEncoder().setPosition(10.5);
    }

    public void shooterRobotInit() {
        slaveShooterMotor.follow(masterShooterMotor);
        hoodMotor.setInverted(true);
        hoodMotor.getEncoder().setPositionConversionFactor(0.65222);
        resetHoodEncoders();
        masterShooterMotor.configVoltageCompSaturation(saturationVoltage);
        slaveShooterMotor.configVoltageCompSaturation(saturationVoltage);
        masterShooterMotor.enableVoltageCompensation(true);
        slaveShooterMotor.enableVoltageCompensation(true);
    }

    public void shooterInit() {
        shooting = false;
        dumpShot = false;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3.0);
    }

    final static float distanceFromTapeToCenterOfHub = 26.6875f;
    final static float distanceFromFenderToCenterOfHub = 33.875f;
    final static float distanceFromFenderToTape = distanceFromFenderToCenterOfHub - distanceFromTapeToCenterOfHub;
    // final static float horizontalDistanceFromLimeLightToShooter = 5.375f;
    // final static float horizontalDistanceFromShooterToFrontOfRobot = 7.25f;
    // final static float horizontalDistanceFromLimelightToFrontOfRobot =
    // horizontalDistanceFromShooterToFrontOfRobot
    // - horizontalDistanceFromLimeLightToShooter;
    final static float horizontalDistanceFromFrontOfRobotToLimelight = 15.0f;
    final static float horizontalDistanceFromFrontOfRobotToShooter = 7.25f;
    final static float horizontalDistanceFromLimelightToShooter = horizontalDistanceFromFrontOfRobotToLimelight
            - horizontalDistanceFromFrontOfRobotToShooter;

    final static float gravity = 386.103f;
    final static float y = 104.0f;
    final static float y0 = 22.5f;
    final static float minimumVelocity = 267.7f;
    final static float maximumVelocity = 405.5f;
    final static float minimumAngle = 0.804f;
    final static float maximumAngle = 1.387f;

    private static float cameraHeight = 40.0f;
    private static float cameraAngle = 45.5f;
    static float angleError = 21.11f;

    public static float getXDistanceFromCenterOfHub(double verticalAngle) {
        return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
                cameraAngle - angleError)))
                + (distanceFromTapeToCenterOfHub - 14.0f) - horizontalDistanceFromLimelightToShooter);
    }

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromFrontOfRobotToLimelight -
    // distanceFromFenderToTape);
    // }

    static float calculatedVelocity = 0.0f;
    static float calculatedAngle = 0.000f;
    public static float shootingCoefficient = 2.561f;

    static boolean setHoodYet = false;
    static boolean goUp = false;
    static boolean goDown = false;
    static boolean calculationBoolean = false;
    public double dumpShotSpeed = 0.425;

    static float tempAngle = 0.000f;
    static float calculatedDistance = 0.000f;

    public static void testVelocity(float x) {

        float velocity = 0.0f;
        for (float i = minimumVelocity; i <= maximumVelocity; i += 1.0f) {
            if ((testAngle(i, x) > minimumAngle) && (testAngle(i, x) < maximumAngle)) {
                velocity = i;
                break;
            }
        }
        calculatedVelocity = velocity;
        calculatedAngle = testAngle(velocity, x);
    }

    public static float testAngle(float velocity, float x) {

        float angle = minimumAngle;

        float[] possibleAngles = new float[(int) ((maximumAngle * 1000.0f) - (minimumAngle * 1000.0f) + 1)];
        for (int i = 0; i < possibleAngles.length; i++) {
            possibleAngles[i] = angle;
            angle = angle + 0.001f;
        }

        for (float possibleAngle : possibleAngles) {
            float slope = slopeOfLine(velocity, possibleAngle, x);
            if (!Double.isNaN(slope)) {
                if (slope > 0.00) {
                    angle = possibleAngle;
                    break;
                }
            }
        }

        while (slopeOfLine(velocity, angle, x) > 0.000f) {
            angle += 0.005f;
        }
        return angle;
    }

    public static float slopeOfLine(float velocity, float possibleAngle, float x) {

        return (float) (((y - x)
                / (((-1.0f) * ((-1.0f) * velocity * Math.sin(possibleAngle) - Math.sqrt(Math.pow(velocity, 2)
                        * Math.pow(Math.sin(possibleAngle), 2) + ((2.0f * gravity) * (y0 - y)))) / gravity)
                        - (x / (velocity * Math.cos(possibleAngle))))));
    }

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromLimelightToFrontOfRobot -
    // distanceFromFenderToTape);
    // }

    public static float shooterWheelLinearVelocityToMotorVelocity(double projectileVelocity) {
        return (float) ((shootingCoefficient
                * (projectileVelocity * (1.0f / (4.0f * Math.PI)) * (1.0f / 1.21) * 2048.0f * (1.0f / 10.0f))));
    }

    public static float shooterWheelLinearVelocityToMotorPercentOutput(double projectileVelocity) {
        // same method except dividing. could change to return above method / 22068.97f
        return (float) ((shootingCoefficient
                * (projectileVelocity * (1.0f / (4.0f * Math.PI)) * (1.0f / 1.21f) * 2048.0f * (1.0f / 10.0f)))
                / 22068.97f);
    }

    double centeringA = 0.001;

    public boolean centerRobotOnTarget() {
        double xAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);

        // if(xAngle < -3.0){
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // -(centeringA * Math.pow(xAngle, 2)), true);
        // return false;
        // } else if(xAngle > 3.0){
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // (centeringA * Math.pow(xAngle, 2)), true);
        // return false;
        // } else {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0), 0.0,
        // true);
        // return true;
        // }

        if (xAngle < -3.0) {
            Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1), Constants.stick.getRawAxis(0),
                    -0.12, true);
            return false;
        } else if (xAngle > 3.0) {
            Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1), Constants.stick.getRawAxis(0),
                    0.12, true);
            return false;
        } else {
            Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1), Constants.stick.getRawAxis(0), 0.0,
                    true);
            return true;
        }

        // if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0)
        // < -20.0) {
        // if (DriverStation.isTeleop()) {

        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, -0.125, true);
        // }
        // return false;
        // } else if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
        // .getDouble(0.0) > 20.0) {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // 0.125,
        // true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, 0.125, true);
        // }
        // return false;
        // } else if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0)
        // < -10.0) {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // -0.125, true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, -0.125, true);
        // }
        // return false;
        // } else if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
        // .getDouble(0.0) > 10.0) {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // 0.125,
        // true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, 0.125, true);
        // }
        // return false;
        // } else if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0)
        // < -6.0) {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // -0.125, true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, -0.125, true);
        // }
        // return false;
        // } else if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
        // .getDouble(0.0) > 6.0) {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // 0.125,
        // true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, 0.125, true);
        // }
        // return false;
        // } else if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0)
        // < -3.0) {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // -0.125, true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, -0.125, true);
        // }
        // return false;
        // } else if
        // (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
        // .getDouble(0.0) > 3.0) {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0),
        // 0.125,
        // true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, 0.125, true);
        // }
        // return false;
        // } else {
        // if (DriverStation.isTeleop()) {
        // Robot.drive.driveTrainByControls(Constants.stick.getRawAxis(1),
        // Constants.stick.getRawAxis(0), 0.0,
        // true);
        // } else {
        // Robot.drive.driveTrainByControls(0.0, 0.0, 0.0, true);
        // }
        // return true;
        // }
    }

    public void shooterIdle(double shooterSpeed) { // does this need to be in the shooter methods or just in
                                                   // teleopPeriodic
        if (!shooting && !dumpShot && Constants.xbox.getPOV() != 0 && Constants.xbox.getPOV() != 180) {
            masterShooterMotor.set(shooterSpeed);
            Robot.intake.indexerMotor.set(0.0);
        }
    }

    Timer shootingTimer = new Timer();

    public void shootInit() {
        setHoodYet = false;
        tempAngle = 0.000f;
        goUp = false;
        goDown = false;
        calculationBoolean = false;
    }

    public void shoot() {

        if (Constants.xbox.getRawButtonPressed(3)) {
            if (!dumpShot) {
                shooting = !shooting;
                if (shooting) {
                    shootInit();
                }
            }
        }

        if (shooting && !dumpShot) {

            if (!calculationBoolean) {
                testVelocity(getXDistanceFromCenterOfHub(
                        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
                calculationBoolean = true;
                shootingTimer.reset();
                shootingTimer.start();
            }

            if (Math.abs(calculatedAngle - tempAngle) > 0.004) {

                if (DriverStation.isTeleop()) {
                    if (shootingTimer.get() > 10.0) {
                        shooting = false;
                    }
                }

                masterShooterMotor.set(ControlMode.PercentOutput,
                        (shooterWheelLinearVelocityToMotorPercentOutput(calculatedVelocity)));

                if ((hoodMotor.getEncoder().getPosition() < (90.0 - Math.toDegrees(calculatedAngle)))
                        && (goDown == false)) {
                    goUp = true;
                }

                if (goUp) {
                    hoodMotor.set(0.1);
                    if ((hoodMotor.getEncoder().getPosition() > (90.0 - Math.toDegrees(calculatedAngle)))) {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                                ControlType.kPosition);
                        setHoodYet = true;
                    }
                }

                if ((hoodMotor.getEncoder().getPosition() > (90.0 - Math.toDegrees(calculatedAngle)))
                        && (goUp == false)) {
                    goDown = true;
                }

                if (goDown) {
                    hoodMotor.set(-0.1);
                    if ((hoodMotor.getEncoder().getPosition() < (90.0 - Math.toDegrees(calculatedAngle)))) {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                                ControlType.kPosition);
                        setHoodYet = true;
                    }
                }

                if ((masterShooterMotor
                        .getSelectedSensorVelocity() > (shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity)
                                * 1.0165))
                        && setHoodYet) {
                    if (DriverStation.isTeleop()) {
                        if (Constants.xbox.getRawAxis(3) > 0.5) {
                            Robot.intake.indexerMotor.set(-0.4);
                        } else {
                            Robot.intake.indexerMotor.set(0.0);
                        }
                    } else {
                        Robot.intake.indexerMotor.set(-0.4);
                    }
                } else {
                    Robot.intake.indexerMotor.set(0.0);
                }
            }
        }
    }

    public void dumpShotLEDs() {
        double currentDistance = getXDistanceFromCenterOfHub(
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0));
        System.out.println(currentDistance);

        if ((currentDistance > (70.0 - 12.0)) && (currentDistance < (70.0 + 12.0))) {
            pwmSparkMax.set(0.71);
        } else {
            pwmSparkMax.set(0.61);
        }
    }

    public void dumpShot() { // 92 inches according to distance method
        // hood all the way down. there's code to put the hood down that could take away
        // later if we locked hood in place

        dumpShotLEDs();

        if (Constants.xbox.getRawButtonPressed(4)) {
            if (!shooting) {
                dumpShot = !dumpShot;
                if (dumpShot) {
                    shootInit();
                    shootingTimer.reset();
                    shootingTimer.start();
                }
            }
        }

        if (dumpShot && !shooting) {

            if (shootingTimer.get() > 10.0) {
                dumpShot = false;
            }

            masterShooterMotor.set(ControlMode.PercentOutput, dumpShotSpeed);

            if ((hoodMotor.getEncoder().getPosition() > 10.8)) { // only goes down to bottom angle of 10.5 degrees if
                                                                 // it's above.
                goDown = true;
            } else {
                setHoodYet = true;
            }

            if (goDown) {
                hoodMotor.set(-0.1);
                if ((hoodMotor.getEncoder().getPosition() < 10.8)) {
                    hoodMotor.set(0.0);
                    setHoodYet = true;
                }
            }

            if ((masterShooterMotor
                        .getSelectedSensorVelocity() > (dumpShotSpeed * 22068.97 * 1.0145)) && setHoodYet) {
                    if (DriverStation.isTeleop()) {
                        if (Constants.xbox.getRawAxis(3) > 0.5) {
                            Robot.intake.indexerMotor.set(-0.4);
                        } else {
                            Robot.intake.indexerMotor.set(0.0);
                        }
                    } else {
                        Robot.intake.indexerMotor.set(-0.4);
                    }
                } else {
                    Robot.intake.indexerMotor.set(0.0);
                }
        }
    }

    public void hoodControl() {
        if (!shooting && !dumpShot) {
            if (Constants.xbox.getPOV() == 270) {
                if (hoodMotor.getEncoder().getPosition() < 10.8) {
                    hoodMotor.set(0.0);
                } else {
                    hoodMotor.set(-0.1);
                }
            } else if (Constants.xbox.getPOV() == 90) {
                hoodMotor.set(0.1);
            } else {
                hoodMotor.set(0.0);
            }
        }
    }

}