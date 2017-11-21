package org.firstinspires.ftc.teamcode;


import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.internal.hardware.TimeWindow;

import java.util.concurrent.locks.Lock;

/**
 * Created by arvindv on 9/18/17.
 */
@SuppressWarnings({"WeakerAccess", "unused"})
@I2cSensor(name = "Pixy", description = "Pixy", xmlTag = "PIXYCAMERA")
public class PixyCamI2CDevice extends I2cDeviceSynchDevice<I2cDeviceSynch> {


    protected PixyCamI2CDevice(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        this.setOptimalReadWindow();

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x54));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();


    }
    protected void writeShort(final PixyRegister reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected void setLED(byte r, byte g, byte b)
    {
        deviceClient.write(PixySyncAddress.LED_SYNC.bVal, new byte[]{r,g,b});
    }

    protected short readShort(PixyRegister reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }
    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                PixyRegister.FIRST_REGISTER.bVal,
                PixyRegister.LAST_REGISTER.bVal - PixyRegister.FIRST_REGISTER.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "PixyCam";
    }

    public enum PixyRegister {
        FIRST_REGISTER(0x00),
        SYNC_R(0x00),
        SYNC_L(0x01),
        CHECKSUM_R(0x02),
        CHECKSUM_L(0x03),
        SIG_R(0x04),
        SIG_L(0x05),
        X_POS_R(0x06),
        X_POS_L(0x07),
        Y_POS_R(0x08),
        Y_POS_L(0x09),
        WIDTH_R(0x0a),
        WIDTH_L(0x0b),
        HEIGHT_R(0x0c),
        HEIGHT_L(0x0d),
        LAST_REGISTER(HEIGHT_L.bVal);

        public int bVal;
        PixyRegister(int bVal) {
            this.bVal = bVal;
        }
    }
    public enum PixySyncAddress {
        WORD_START(0xaa55),
        WORD_START_CC(0xaa56),
        WORD_START_X(0x55aa),
        CAMERA_BRIGHTNESS_SYNC(0xfe00),
        LED_SYNC(0xfd00);
        public int bVal;
        PixySyncAddress (int bVal) {
            this.bVal = bVal;
        }
    }
}
