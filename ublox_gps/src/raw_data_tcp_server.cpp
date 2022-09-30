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



#include "ublox_gps/raw_data_server.h"


using namespace ublox_node;


RawDataStreamServer::RawDataStreamServer(uint16_t port) :pnh_(ros::NodeHandle("~")), io_service_(), p_socket_(nullptr), p_acceptor_(
        nullptr) {
    tcp_port_ = port;
    enable_server_ = false;
}

void RawDataStreamServer::startAccept() {
    p_acceptor_->async_accept(
    [this](boost::system::error_code ec, tcp::socket socket)
            {
                if (!ec)
                {
                    ROS_INFO_STREAM("New connection on ublox debug TCP socket");
                    if(p_socket_ && p_socket_->is_open()) {
                        p_socket_->close();
                    }
                    this->p_socket_ = std::make_shared<tcp::socket>(std::move(socket));
                } else {
                    ROS_WARN("Error during client debug TCP client connection.");
                }

                startAccept();
            });
}

void RawDataStreamServer::getRosParams(void) {
    pnh_.param("raw_data_server/enable", enable_server_, false);
}

bool RawDataStreamServer::isEnabled(void) {
    return enable_server_;
}

void RawDataStreamServer::initialize(void) {
    ROS_INFO("Initializing raw data server");
    p_acceptor_ = std::make_shared<tcp::acceptor>(tcp::acceptor (io_service_,tcp::endpoint(tcp::v4(), tcp_port_)));
    startAccept();
    background_thread_.reset(new boost::thread([this] {
        io_service_.run();
    }));
}

void RawDataStreamServer::ubloxCallback(const unsigned char *data, const std::size_t size) {
    if(p_socket_) {
        try {
            boost::asio::write(*p_socket_, boost::asio::buffer(data, size));
        } catch (std::exception &e) {
            ROS_WARN("Error during stream write, closing connection");
            p_socket_->close();
            p_socket_ = nullptr;
        }
    }
}

RawDataStreamServer::~RawDataStreamServer() {
    io_service_.stop();
    background_thread_->join();
}


