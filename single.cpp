#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class L3G4200D {
public:
    L3G4200D(int bus, int address);
    ~L3G4200D();

    bool initialize();
    bool readData(int& x, int& y, int& z);

private:
    int i2cFile;
    int deviceAddress;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
};

L3G4200D::L3G4200D(int bus, int address) : deviceAddress(address) {
    char filename[20];
    sprintf(filename, "/dev/i2c-%d", bus);
    i2cFile = open(filename, O_RDWR);
    if (i2cFile < 0) {
        std::cerr << "Error opening I2C bus." << std::endl;
    }
}

L3G4200D::~L3G4200D() {
    close(i2cFile);
}

bool L3G4200D::initialize() {
    if (ioctl(i2cFile, I2C_SLAVE, deviceAddress) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        return false;
    }

    // Initialize the L3G4200D sensor (assumes default configuration)
    if (!writeRegister(0x20, 0x0F)) { // CTRL_REG1 (normal mode, 100Hz)
        std::cerr << "Failed to initialize L3G4200D." << std::endl;
        return false;
    }

    return true;
}

bool L3G4200D::readData(int& x, int& y, int& z) {
    uint8_t buffer[6];

    if (readRegister(0x28, buffer[0]) && // OUT_X_L
        readRegister(0x29, buffer[1]) && // OUT_X_H
        readRegister(0x2A, buffer[2]) && // OUT_Y_L
        readRegister(0x2B, buffer[3]) && // OUT_Y_H
        readRegister(0x2C, buffer[4]) && // OUT_Z_L
        readRegister(0x2D, buffer[5])    // OUT_Z_H
    ) {
        x = (buffer[1] << 8) | buffer[0];
        y = (buffer[3] << 8) | buffer[2];
        z = (buffer[5] << 8) | buffer[4];
        return true;
    }

    return false;
}

bool L3G4200D::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;

    if (write(i2cFile, buffer, 2) != 2) {
        std::cerr << "Failed to write to the I2C bus." << std::endl;
        return false;
    }

    return true;
}

bool L3G4200D::readRegister(uint8_t reg, uint8_t& value) {
    if (write(i2cFile, &reg, 1) != 1) {
        std::cerr << "Failed to write to the I2C bus." << std::endl;
        return false;
    }

    if (read(i2cFile, &value, 1) != 1) {
        std::cerr << "Failed to read from the I2C bus." << std::endl;
        return false;
    }

    return true;
}

int main() {
    // Initialize the L3G4200D sensor on I2C bus 1, device address 0x69
    L3G4200D gyro(1, 0x69);
    if (!gyro.initialize()) {
        return 1;
    }

    // Read and print gyroscope data
    int x, y, z;
    while (true) {
        if (gyro.readData(x, y, z)) {
            std::cout << "X: " << x << "  Y: " << y << "  Z: " << z << std::endl;
        } else {
            std::cerr << "Failed to read data from L3G4200D." << std::endl;
            return 1;
        }

        // Add a delay between readings
        usleep(100000); // 100ms delay
    }

    return 0;
}
