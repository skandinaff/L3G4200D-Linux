/*
L3G4200D.cpp - Class file for the L3G4200D Triple Axis Gyroscope Arduino Library.

Version: 1.3.3
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include "L3G4200D.h"



uint8_t L3G4200D::i2cread(void)
{
    uint8_t data;
    if (read(i2cFile, &data, 1) != 1) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }
    return data;
}


void L3G4200D::i2cwrite(uint8_t x)
{
    if (write(i2cFile, &x, 1) != 1) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }
}


bool L3G4200D::begin(l3g4200d_dps_t scale, l3g4200d_odrbw_t odrbw)
{
    // Reset calibrate values
    d.XAxis = 0;
    d.YAxis = 0;
    d.ZAxis = 0;
    useCalibrate = false;

    // Reset threshold values
    t.XAxis = 0;
    t.YAxis = 0;
    t.ZAxis = 0;
    actualThreshold = 0;

    i2cFile = open(I2C_DEV, O_RDWR);

    if (i2cFile < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }

    // Check L3G4200D Who Am I Register
    uint8_t whoami;
    whoami = fastRegister8(L3G4200D_REG_WHO_AM_I);
    if (whoami != 0xD3)
    {
	    return false;
    } else {
        printf("Whoami result: %d\n", whoami);
    }

    // Enable all axis and setup normal mode + Output Data Range & Bandwidth
    uint8_t reg1 = 0x00;
    reg1 |= 0x0F; // Enable all axis and setup normal mode
    reg1 |= (odrbw << 4); // Set output data rate & bandwidh
    writeRegister8(L3G4200D_REG_CTRL_REG1, reg1);

    // Disable high pass filter
    writeRegister8(L3G4200D_REG_CTRL_REG2, 0x00);

    // Generata data ready interrupt on INT2
    //writeRegister8(L3G4200D_REG_CTRL_REG3, 0x08);

    // Set full scale selection in continous mode
    //writeRegister8(L3G4200D_REG_CTRL_REG4, scale << 4);

    switch(scale)
    {
	case L3G4200D_SCALE_250DPS:
	    dpsPerDigit = .00875f;
	    break;
	case L3G4200D_SCALE_500DPS:
	    dpsPerDigit = .0175f;
	    break;
	case L3G4200D_SCALE_2000DPS:
	    dpsPerDigit = .07f;
	    break;
	default:
	    break;
    }

    // Boot in normal mode, disable FIFO, HPF disabled
    writeRegister8(L3G4200D_REG_CTRL_REG5, 0x00);

    return true;
}

// Get current scale
l3g4200d_dps_t L3G4200D::getScale(void)
{
    return (l3g4200d_dps_t)((readRegister8(L3G4200D_REG_CTRL_REG4) >> 4) & 0x03);
}


// Get current output data range and bandwidth
l3g4200d_odrbw_t L3G4200D::getOdrBw(void)
{
    return (l3g4200d_odrbw_t)((readRegister8(L3G4200D_REG_CTRL_REG1) >> 4) & 0x0F);
}

// Calibrate algorithm
void L3G4200D::calibrate(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
        readRaw();
        sumX += r.XAxis;
        sumY += r.YAxis;
        sumZ += r.ZAxis;

        sigmaX += r.XAxis * r.XAxis;
        sigmaY += r.YAxis * r.YAxis;
        sigmaZ += r.ZAxis * r.ZAxis;
        
        usleep(5000);
    }

    // Calculate delta vectors
    d.XAxis = sumX / samples;
    d.YAxis = sumY / samples;
    d.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    thresholdX = sqrt((sigmaX / samples) - (d.XAxis * d.XAxis));
    thresholdY = sqrt((sigmaY / samples) - (d.YAxis * d.YAxis));
    thresholdZ = sqrt((sigmaZ / samples) - (d.ZAxis * d.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
	setThreshold(actualThreshold);
    }
}

// Get current threshold value
uint8_t L3G4200D::getThreshold(void)
{
    return actualThreshold;
}

// Set treshold value
void L3G4200D::setThreshold(uint8_t multiple)
{
    if (multiple > 0)
    {
	// If not calibrated, need calibrate
	if (!useCalibrate)
	{
	    calibrate();
	}
	
	// Calculate threshold vectors
	t.XAxis = thresholdX * multiple;
	t.YAxis = thresholdY * multiple;
	t.ZAxis = thresholdZ * multiple;
    } else
    {
	// No threshold
	t.XAxis = 0;
	t.YAxis = 0;
	t.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}

// Write 8-bit to register
void L3G4200D::writeRegister8(uint8_t reg, uint8_t value)
{

    if (ioctl(i2cFile, I2C_SLAVE, L3G4200D_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    i2cwrite(reg);
    i2cwrite(value);

    //TODO: there was End Trasmission 
}

// Fast read 8-bit from register
uint8_t L3G4200D::fastRegister8(uint8_t reg)
{
    uint8_t ret_val;
    if (ioctl(i2cFile, I2C_SLAVE, L3G4200D_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    i2cwrite(reg);

    if (read(i2cFile, &ret_val, 1) != 1) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }

    return ret_val;
}

// Read 8-bit from register
uint8_t L3G4200D::readRegister8(uint8_t reg)
{
    if (ioctl(i2cFile, I2C_SLAVE, L3G4200D_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    i2cwrite(reg);

    usleep(10000); 

    if (read(i2cFile, &reg, 1) != 1) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }

    return reg;
}
// L3G4200D Temperature sensor output change vs temperature: -1digit/degrCelsius (data representation: 2's complement).
// Value represents difference respect to a reference not specified value.
// So temperature sensor can be used to measure temperature variations: temperarture sensor isn't suitable to return absolute temperatures measures.
// If you run two sequential measures and differentiate them you can get temperature variation.
// This also means that two devices in the same temp conditions can return different outputs.
// Finally, you can use this info to compensate drifts due to temperature changes.
uint8_t L3G4200D::readTemperature(void)
{
    return readRegister8(L3G4200D_REG_OUT_TEMP);
}

// Read raw values
Vector L3G4200D::readRaw()
{


    uint8_t reg = L3G4200D_REG_OUT_X_L | (1 << 7);

    if (ioctl(i2cFile, I2C_SLAVE, L3G4200D_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    i2cwrite(reg);


    if (read(i2cFile, &buf, 6) != 6) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }

    r.XAxis = (int16_t)(buf[1] << 8 | buf[0]);
    r.YAxis = (int16_t)(buf[3] << 8 | buf[2]);
    r.ZAxis = (int16_t)(buf[5] << 8 | buf[4]);

    return r;
}

// Read normalized values
Vector L3G4200D::readNormalize()
{
    readRaw();

    if (useCalibrate)
    {
	n.XAxis = (r.XAxis - d.XAxis) * dpsPerDigit;
	n.YAxis = (r.YAxis - d.YAxis) * dpsPerDigit;
	n.ZAxis = (r.ZAxis - d.ZAxis) * dpsPerDigit;
    } else
    {
	n.XAxis = r.XAxis * dpsPerDigit;
	n.YAxis = r.YAxis * dpsPerDigit;
	n.ZAxis = r.ZAxis * dpsPerDigit;
    }

    if (actualThreshold > 0)
    {
	if (abs(n.XAxis) < t.XAxis) n.XAxis = 0;
	if (abs(n.YAxis) < t.YAxis) n.YAxis = 0;
	if (abs(n.ZAxis) < t.ZAxis) n.ZAxis = 0;
    }

    return n;
}