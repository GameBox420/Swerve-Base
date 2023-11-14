package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {


    NetworkTable LIMELIGHT = NetworkTableInstance.getDefault().getTable("/limelight");
    NetworkTableEntry LIMELIGHT_TX = LIMELIGHT.getEntry("tx");
    NetworkTableEntry LIMELIGHT_TY = LIMELIGHT.getEntry("ty");
    NetworkTableEntry LIMELIGHT_TA = LIMELIGHT.getEntry("ta");
    NetworkTableEntry LIMELIGHT_TV = LIMELIGHT.getEntry("tv");

    public LimeLight() {}
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LimeLight Has Target",getLimeLightTV());
        SmartDashboard.putNumber("LimeLight X Delta", getLimeLightTX());
        SmartDashboard.putNumber("LimeLight Y Delta", getLimeLightTY());
        SmartDashboard.putNumber("LimeLight Target Area", getLimeLightTA());
    }

    public double getLimeLightTX() {
        return LIMELIGHT_TX.getDouble(0.0);
    }
    public double getLimeLightTY() {
        return LIMELIGHT_TY.getDouble(0.0);
    }
    public double getLimeLightTA() {
        return LIMELIGHT_TA.getDouble(0.0);
    }
    public boolean getLimeLightTV() {
        return LIMELIGHT_TV.getBoolean(false);
    }

}