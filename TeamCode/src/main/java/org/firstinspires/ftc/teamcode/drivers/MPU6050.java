package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(
        name = "MPU6050 IMU",
        xmlTag = "MPU6050",
        description = "MPU6050 6-DOF accelerometer + gyro"
)
public class MPU6050 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // Default I²C address when AD0 pin is LOW
    public static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x68);
    // If your board ties AD0 HIGH, change to 0x69:
    // public static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x69);

    // MPU6050 register map (subset)
    public enum Register {
        SMPLRT_DIV(0x19),
        CONFIG(0x1A),
        GYRO_CONFIG(0x1B),
        ACCEL_CONFIG(0x1C),
        INT_ENABLE(0x38),
        ACCEL_XOUT_H(0x3B),
        PWR_MGMT_1(0x6B);

        public final int bVal;
        Register(int bVal) { this.bVal = bVal; }
    }

    public static class Sample {
        // Raw ADC units
        public short axRaw, ayRaw, azRaw;
        public short gxRaw, gyRaw, gzRaw;
        public short tempRaw;

        // Converted units (assuming ±2g and ±250°/s)
        public float axG, ayG, azG;        // g
        public float gxDps, gyDps, gzDps;  // deg/s
        public float tempC;
    }

    public MPU6050(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MPU6050 6DOF IMU";
    }

    // Called when the SDK initializes the device
    @Override
    protected synchronized boolean doInitialize() {
        // Wake up device (clear sleep bit)
        write8(Register.PWR_MGMT_1, (byte) 0x00);

        // Optional: setup low-pass filter, sample rate, scale, etc.
        // Sample rate divider: sample_rate = gyro_output_rate / (1 + SMPLRT_DIV)
        write8(Register.SMPLRT_DIV, (byte) 0x07); // ~1kHz / (1+7) = 125Hz

        // CONFIG: DLPF (low-pass filter). 0x03 is a common choice.
        write8(Register.CONFIG, (byte) 0x03);

        // GYRO_CONFIG: ±250°/s (0x00) – easiest to start with
        write8(Register.GYRO_CONFIG, (byte) 0x00);

        // ACCEL_CONFIG: ±2g (0x00)
        write8(Register.ACCEL_CONFIG, (byte) 0x00);

        // Enable data ready interrupt if you ever want to use INT pin
        write8(Register.INT_ENABLE, (byte) 0x01);

        return true;
    }

    // Convenience wrappers
    protected void write8(Register reg, byte value) {
        deviceClient.write8(reg.bVal, value);
    }

    protected byte read8(Register reg) {
        return deviceClient.read8(reg.bVal);
    }

    // Helper for big-endian bytes -> signed 16-bit
    private short bytesToShort(byte high, byte low) {
        return (short) (((high & 0xFF) << 8) | (low & 0xFF));
    }

    /** Read accel + gyro + temp in one shot */
    public synchronized Sample readSample() {
        // 14 bytes starting at ACCEL_XOUT_H:
        // axH, axL, ayH, ayL, azH, azL, tempH, tempL, gxH, gxL, gyH, gyL, gzH, gzL
        byte[] buf = deviceClient.read(Register.ACCEL_XOUT_H.bVal, 14);

        Sample s = new Sample();

        s.axRaw   = bytesToShort(buf[0],  buf[1]);
        s.ayRaw   = bytesToShort(buf[2],  buf[3]);
        s.azRaw   = bytesToShort(buf[4],  buf[5]);
        s.tempRaw = bytesToShort(buf[6],  buf[7]);
        s.gxRaw   = bytesToShort(buf[8],  buf[9]);
        s.gyRaw   = bytesToShort(buf[10], buf[11]);
        s.gzRaw   = bytesToShort(buf[12], buf[13]);

        // Datasheet sensitivities for default ranges:
        // Accel ±2g  => 16384 LSB/g
        // Gyro ±250°/s => 131 LSB/(°/s)
        s.axG = s.axRaw / 16384.0f;
        s.ayG = s.ayRaw / 16384.0f;
        s.azG = s.azRaw / 16384.0f;

        s.gxDps = s.gxRaw / 131.0f;
        s.gyDps = s.gyRaw / 131.0f;
        s.gzDps = s.gzRaw / 131.0f;

        // Temperature formula from datasheet:
        // Temp in °C = (tempRaw / 340) + 36.53
        s.tempC = (s.tempRaw / 340.0f) + 36.53f;

        return s;
    }
}
