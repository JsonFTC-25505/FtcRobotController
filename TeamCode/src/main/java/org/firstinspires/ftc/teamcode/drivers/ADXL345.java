package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(
        name = "ADXL345 Accelerometer",
        xmlTag = "adxl345")
public class ADXL345 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr I2C_ADDR = I2cAddr.create7bit(0x53); // or 0x1D

    // Key registers from datasheet
    private static final int REG_BW_RATE     = 0x2C;
    private static final int REG_POWER_CTL   = 0x2D;
    private static final int REG_DATA_FORMAT = 0x31;
    private static final int REG_DATAX0      = 0x32;  // X0..Z1 = 0x32–0x37

    // Constructor used by the FTC hardware map (recommended)
    public ADXL345(I2cDeviceSynch deviceClient) {
        this(deviceClient, true);
    }

    // Main constructor
    public ADXL345(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2C_ADDR);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "ADXL345 3-Axis Accelerometer";
    }

    @Override
    protected synchronized boolean doInitialize() {
        // 100 Hz output data rate (0x0A) – optional but nice
        deviceClient.write8(REG_BW_RATE, 0x0A);

        // Full resolution, ±2g range: FULL_RES=1, RANGE=00 → 0x08
        deviceClient.write8(REG_DATA_FORMAT, 0x08);

        // Measurement mode: set MEASURE bit (bit 3) in POWER_CTL → 0x08
        deviceClient.write8(REG_POWER_CTL, 0x08);

        return true;
    }

    /** Read acceleration in g on all three axes */
    public double[] getAccel() {
        byte[] raw = deviceClient.read(REG_DATAX0, 6);

        int x = (short) ((raw[1] << 8) | (raw[0] & 0xFF));
        int y = (short) ((raw[3] << 8) | (raw[2] & 0xFF));
        int z = (short) ((raw[5] << 8) | (raw[4] & 0xFF));

        // In full-resolution ±2g mode, scale is 4 mg/LSB = 0.004 g/LSB
        double scale = 0.004;
        return new double[]{x * scale, y * scale, z * scale};
    }
}
