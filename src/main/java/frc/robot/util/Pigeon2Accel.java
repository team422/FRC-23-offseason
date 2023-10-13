package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants.DriveConstants.Pigeon2AccelConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class Pigeon2Accel implements Accelerometer {
    private Pigeon2 m_pigeon;

    public Pigeon2Accel(Pigeon2 pigeon) {
        m_pigeon = pigeon;
    }

    public Pigeon2Accel(int port) {
        this(new Pigeon2(port));
    }

    @Override
    public void setRange(Range range) {
        DriverStation.reportWarning("Cannot set range for Pigeon2 Accelerometer", false);
    }

    @Override
    public double getX() {
        return getAccelData(Pigeon2AccelConstants.kX);
    }

    @Override
    public double getY() {
        return getAccelData(Pigeon2AccelConstants.kY);
    }

    @Override
    public double getZ() {
        return getAccelData(Pigeon2AccelConstants.kZ);
    }

    private short[] getRawAccelData() {
        short[] data = new short[3];
        m_pigeon.getBiasedAccelerometer(data);
        return data;
    }

    private short getRawAccelData(int axis) {
        return getRawAccelData()[axis];
    }

    @SuppressWarnings("unused") // the warning was annoying me, remove this if this method is ever used lol
    private double[] getAccelData() {
        short[] raw = getRawAccelData();
        double[] data = new double[3];
        for (int i = 0; i < 3; i++) {
            data[i] = convertToActual(raw[i]);
        }
        return data;
    }

    private double getAccelData(int axis) {
        return convertToActual(getRawAccelData(axis));
    }

    private static double convertToActual(short raw) {
        return fixedToDouble(raw, Pigeon2AccelConstants.kFixedShift) * Pigeon2AccelConstants.kFixedShift;
    }

    private static double fixedToDouble(short fixed, int shift) {
        return ((double) fixed) / (1 << shift);
    }
}