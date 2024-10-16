#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <thread>

#include "spihandler.h"

using namespace spihandler;

namespace
{
// Register definitions
enum class SPISensRegs : uint8_t
{
    ID,
    CTRL,
    TEMPERATURE
};
// Enable value
constexpr uint8_t CTRL_ENABLE = 0x01;
// Command operation flag: 0 - read, 1 - write
constexpr uint8_t CMD_OP_WRITE = 0x80;
// Command register index shift
constexpr uint8_t CMD_REG_SHIFT = 4;
}; // namespace

//-----------------------------------------------------------------------------

SPIHandler::SPIHandler(std::string path) : m_path{path}, m_running{false}
{
}

//-----------------------------------------------------------------------------

SPIHandler::~SPIHandler()
{
}

//-----------------------------------------------------------------------------

int8_t SPIHandler::init(void)
{
    // Open FD
    m_fd = open(m_path.c_str(), O_RDWR);
    if (m_fd == -1)
    {
        return m_fd;
    }

    // NOTE: Usually, here we would configure the SPI mode, CPOL/CPHA, speed, etc.
    // However, since this is running in QEMU, none of it is necessary.

    // Enable the device
    if (!write_reg(static_cast<uint8_t>(SPISensRegs::CTRL), CTRL_ENABLE))
    {
        return -2;
    }

    return 0;
}

//-----------------------------------------------------------------------------

void SPIHandler::deinit(void)
{
    if (!write_reg(static_cast<uint8_t>(SPISensRegs::CTRL), 0))
    {
        std::cerr << "Problem disabling device" << std::endl;
    }
}

//-----------------------------------------------------------------------------

void SPIHandler::runThread(void)
{
    if (!m_thread.joinable())
    {
        m_thread = std::thread(&SPIHandler::do_read, this);
    }
}

//-----------------------------------------------------------------------------

void SPIHandler::stopThread(void)
{
    m_running = false;

    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

//-----------------------------------------------------------------------------

void SPIHandler::do_read(void)
{
    std::chrono::system_clock::time_point current_time;
    uint8_t value;

    m_running = true;

    while (m_running)
    {
        // Read current time so we know when to timeout
        current_time = std::chrono::system_clock::now();

        // read SPI device and print
        if (!read_reg(static_cast<uint8_t>(SPISensRegs::TEMPERATURE), value))
        {
            m_running = false;
            break;
        }

        std::cout << "Measured " << std::setprecision(3) << (value / 2.) << std::endl;

        // sleep_until
        std::this_thread::sleep_until(current_time + std::chrono::seconds(1));
    }
}

//-----------------------------------------------------------------------------

bool SPIHandler::write_reg(uint8_t reg_nr, uint8_t val)
{
    struct spi_ioc_transfer xfer[1] = {0};

    // Command byte
    uint8_t cmd_byte = CMD_OP_WRITE | (reg_nr << CMD_REG_SHIFT);
    uint8_t data[2];
    data[0] = cmd_byte;
    data[1] = val;
    xfer[0].tx_buf = (__u64)data;      // output buffer
    xfer[0].rx_buf = (__u64)data;      // input buffer
    xfer[0].len = (__u32)sizeof(data); // length of data to read

    int retv = ioctl(m_fd, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0)
    {
        std::cout << "error in spi_write_reg8(): ioctl(SPI_IOC_MESSAGE(1)) return" << std::endl;
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------

bool SPIHandler::read_reg(uint8_t reg_nr, uint8_t &val)
{
    struct spi_ioc_transfer xfer[1] = {0};

    // Command byte
    uint8_t cmd_byte = reg_nr << CMD_REG_SHIFT;
    uint8_t data[2];
    data[0] = cmd_byte;
    data[1] = 0x00;
    xfer[0].tx_buf = (__u64)data;      // output buffer
    xfer[0].rx_buf = (__u64)data;      // input buffer
    xfer[0].len = (__u32)sizeof(data); // length of data to read

    int retv = ioctl(m_fd, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0)
    {
        std::cout << "error in spi_read_reg8(): ioctl(SPI_IOC_MESSAGE(1))" << std::endl;
        return false;
    }
    val = data[1];
    return true;
}
