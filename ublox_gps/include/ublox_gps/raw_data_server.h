//==============================================================================
// Copyright (c) 2022, Clemens Elflein
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_RAW_DATA_SERVER_H
#define UBLOX_RAW_DATA_SERVER_H

#include <vector>
#include <set>
#include <fstream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

/**
 * @namespace ublox_node
 * This namespace is for the ROS u-blox node and handles anything regarding
 * ROS parameters, message passing, diagnostics, etc.
 */
namespace ublox_node {
    using boost::asio::ip::tcp;
/**
 * @brief Implements a raw TCP server for u-center to connect to.
 */
class RawDataStreamServer {
  public:


    /**
     * @brief Constructor.
     * Initialises variables and the nodehandle.
     */
    RawDataStreamServer(uint16_t port = 4242);

    /**
     * @brief Get the raw data stream parameters.
     */
    void getRosParams(void);

    /**
     * @brief Returns the if raw data streaming is enabled.
     */
    bool isEnabled(void);

    /**
     * @brief Initializes the server
     */
    void initialize(void);


    /**
     * @brief Callback function which handles raw data.
     * @param data the buffer of u-blox messages to process
     * @param size the size of the buffer
     */
    void ubloxCallback(const unsigned char* data,
     const std::size_t size);

    ~RawDataStreamServer();

  private:
    boost::shared_ptr<boost::thread> background_thread_;

    //! True, if server should be enabled
    bool enable_server_;

    //! Waits for a client
    void startAccept();

    boost::asio::io_service io_service_;
    std::shared_ptr<tcp::acceptor> p_acceptor_;
    std::shared_ptr<tcp::socket> p_socket_;

    //! Port for the TCP server
    uint16_t tcp_port_;
    //! ROS private node handle (for params)
    ros::NodeHandle pnh_;
};

}

#endif
