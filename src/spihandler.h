/**
 * @file SPIHandler.h
 * @author Strahinja Jankovic (strahinja.p.jankovic[at]gmail.com)
 * @details Implementation of SPI userspace handling
 */

#ifndef _SPI_HANDLER_H_
#define _SPI_HANDLER_H_

#include <string>
#include <thread>

namespace spihandler
{

class SPIHandler
{
public:
    /**
     * Constructor
     *
     * @param path - path to SPI device
     */
    explicit SPIHandler(std::string path);

    /* Disable copy constructor and assignment. */
    SPIHandler(const SPIHandler &) = delete;
    SPIHandler &operator=(const SPIHandler &) = delete;

    /** Destructor. */
    virtual ~SPIHandler();

    /**
     * init
     *
     * Initialize SPI device.
     */
    int8_t init(void);

    /**
     * deinit
     *
     * Disable SPI device.
     */
    void deinit(void);

    /**
     * runThread
     *
     * Start executing thread.
     */
    void runThread(void);

    /**
     * stopThread
     *
     * Stop executing thread.
     */
    void stopThread(void);

private:
    /**
     * do_read
     *
     * Thread that periodically reads SPI temperature data.
     */
    void do_read(void);

    /**
     * write_reg
     *
     * Helper function to write register value.
     */
    bool write_reg(uint8_t reg_nr, uint8_t val);

    /**
     * read_reg
     *
     * Helper function to read register value
     */
    bool read_reg(uint8_t reg_nr, uint8_t &val);

    /** Path to SPI device. */
    std::string m_path;

    /** Thread handle. */
    std::thread m_thread;

    /** Flag used to control thread execution. */
    bool m_running;

    /** SPI dev file descriptor. */
    int m_fd;
};

} // namespace spihandler

#endif // _SPI_HANDLER_H_
