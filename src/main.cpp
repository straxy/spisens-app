#include "spihandler.h"
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <thread>

using namespace spihandler;
namespace fs = std::filesystem;

namespace
{
static bool signalReceived = false;
};

class SignalHandler
{
public:
    SignalHandler()
    {
    }
    ~SignalHandler()
    {
    }
    static void setupSignals();

private:
    static void handleSignal(int sig);
};

void SignalHandler::setupSignals()
{
    std::signal(SIGINT, SignalHandler::handleSignal);
}

void SignalHandler::handleSignal(__attribute__((unused)) int sig)
{
    signalReceived = true;
}

int main(int argc, char *argv[])
{
    int8_t retval = 0;

    // check that valid dev path is passed
    if ((argc != 2) || !fs::is_character_file(argv[1]))
    {
        std::cout << "/dev path to spidev device must be passed" << std::endl;
        exit(-1);
    }

    // Configure signal handling
    SignalHandler::setupSignals();

    // Create a spi handler
    SPIHandler daemon(argv[1]);

    // Initialize the device
    retval = daemon.init();
    if (retval != 0)
    {
        std::cout << "Error initializing SPI device " << int(retval) << std::endl;
        exit(-2);
    }

    // Start poller thread
    daemon.runThread();

    while (!signalReceived)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Stop and join thread
    daemon.stopThread();

    daemon.deinit();

    return 0;
}
