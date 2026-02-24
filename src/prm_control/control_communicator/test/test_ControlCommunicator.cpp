#include <gtest/gtest.h>
#include "ControlCommunicator.h"
#include <list>

class ControlCommunicatorTest : public ::testing::Test
{
protected:
    const char *port = nullptr;
    int port_fd = -1;
    bool PORT_EXISTS = false;

    void SetUp() override
    {
        // Loop through possible UART ports to fnd one we can access and open
        const std::list<const char *> ports = {"/dev/ttyTHS0", "/dev/ttyTHS1", "/dev/ttyTHS2"};
        for (const char *p : ports)
        {
            if (access(p, F_OK) != -1 && open(p, O_RDWR | O_NOCTTY | O_NONBLOCK) != -1)
            {
                port = p;
                PORT_EXISTS = true;
                break;
            }
        }
    }

    void TearDown() override
    {
    }
};

TEST_F(ControlCommunicatorTest, test_start_uart_connection)
{
    ControlCommunicator control_communicator;
    if (PORT_EXISTS)
    {
        EXPECT_TRUE(control_communicator.start_uart_connection(port));
        EXPECT_NE(control_communicator.port_fd, -1);    // Port should be opened

        /**
         * Check UART configuration
         */
        struct termios tty;
        int port_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(port_fd, -1); // Port should be opened

        if (tcgetattr(port_fd, &tty) != 0)
        {
            close(port_fd);
            FAIL();
        }

        // Check baud rate
        EXPECT_EQ(cfgetispeed(&tty), B1152000);
        EXPECT_EQ(cfgetospeed(&tty), B1152000);

        // Check VMIN and VTIME
        EXPECT_EQ(tty.c_cc[VMIN], 1);
        EXPECT_EQ(tty.c_cc[VTIME], 10);
    }
    else
    {
        // Neither port exists on dev server or local machine, since no serial device
        EXPECT_FALSE(control_communicator.start_uart_connection(port));
    }
}

TEST_F(ControlCommunicatorTest, test_read_uart_values)
{
    ControlCommunicator control_communicator;
    if (PORT_EXISTS)
    {
        EXPECT_TRUE(control_communicator.start_uart_connection(port));
        EXPECT_NE(control_communicator.port_fd, -1);    // Port should be opened
        EXPECT_TRUE(control_communicator.is_connected); // Connection should be established

        PackageIn package;
        bool result = false;
        for (int i = 0; i < 100; ++i)
        {
            result = control_communicator.read_uart(open(port, O_RDWR | O_NOCTTY | O_NONBLOCK), package, port);
            if (result)
            {
                break; // Exit loop if read is successful
            }
        }
        EXPECT_TRUE(result); // Ensure at least one successful read

        // Validate the package data
        EXPECT_EQ(package.head, 0xAA);          // Check head byte
        EXPECT_EQ(package.ref_flags & 0x01, 0); // Check ref_flags (LSB should be 0 since match not started)
        EXPECT_LT(package.pitch, M_PI);         // Check pitch value
        EXPECT_GT(package.pitch, -M_PI);        // Check pitch value
        EXPECT_LT(package.pitch_vel, M_PI);     // Check pitch velocity
        EXPECT_GT(package.pitch_vel, -M_PI);    // Check pitch velocity
        EXPECT_LT(package.yaw_vel, M_PI);       // Check yaw velocity
        EXPECT_GT(package.yaw_vel, -M_PI);      // Check yaw velocity

        // package x field should exist and be a number
        EXPECT_NE(package.x, NAN);           // Check x position
        EXPECT_NE(package.y, NAN);           // Check y position
        EXPECT_NE(package.orientation, NAN); // Check orientation
        EXPECT_NE(package.x_vel, NAN);       // Check x velocity
        EXPECT_NE(package.y_vel, NAN);       // Check y velocity
    }
    else
    {
        // Neither port exists on dev server or local machine, since no serial device
        EXPECT_FALSE(control_communicator.start_uart_connection(port));
    }
}

TEST_F(ControlCommunicatorTest, test_compute_aim)
{
    ControlCommunicator control_communicator;
    float target_x = 40.0;     // mm 
    float target_y = 180.0;       // mm
    float target_z = 1500.0;    // mm
    float bullet_speed = 16.0;  // m/s
    float yaw, pitch;
    bool impossible;

    control_communicator.compute_aim(bullet_speed, target_x, target_y, target_z, yaw, pitch, impossible);
    EXPECT_EQ(impossible, false);   // Shot should be possible
    EXPECT_NEAR(yaw, 0.0, 2.0);     // Yaw = 0.0 since no X and Y offset
    EXPECT_NEAR(pitch, 12.0, 1.50);  // Check pitch angle
}

TEST_F(ControlCommunicatorTest, test_aim)
{
    ControlCommunicator control_communicator;
    if (PORT_EXISTS)
    {
        EXPECT_TRUE(control_communicator.start_uart_connection(port));
        EXPECT_NE(control_communicator.port_fd, -1);    // Port should be opened
        EXPECT_TRUE(control_communicator.is_connected); // Connection should be established

        float target_x;
        float target_y;
        float target_z;
        float bullet_speed;
        float yaw, pitch;
        bool impossible;

        // Now generate random values and test sending to board
        int NUM_ITERS = 100;
        for (int i = 0; i < NUM_ITERS; ++i)
        {
            target_x = 10.0 + rand() % 2000; // mm
            target_y = 10.0 + rand() % 2000; // mm
            target_z = 10.0 + rand() % 2000; // mm
            bullet_speed = rand() % 20 + 1; // m/s

            int bytes_written = control_communicator.aim(bullet_speed, target_x, target_y, target_z, yaw, pitch, impossible);
            EXPECT_EQ(bytes_written, sizeof(PackageOut)); // Ensure complete package is written
        }
    }
    else
    {
        // Neither port exists on dev server or local machine, since no serial device
        EXPECT_FALSE(control_communicator.start_uart_connection(port));
    }
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}