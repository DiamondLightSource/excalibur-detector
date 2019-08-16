#include <string>

#include "ExcaliburFrameSimulatorPlugin.h"
#include "ExcaliburDefinitions.h"

#include <cstdlib>
#include <time.h>
#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>

#include "version.h"

namespace FrameSimulator {

    /** Construct an ExcaliburFrameSimulatorPlugin
    * setup an instance of the logger
    * initialises data and frame counts
    */
    ExcaliburFrameSimulatorPlugin::ExcaliburFrameSimulatorPlugin() : FrameSimulatorPluginUDP() {

        // Setup logging for the class
        logger_ = Logger::getLogger("FS.ExcaliburFrameSimulatorPlugin");
        logger_->setLevel(Level::getAll());

        total_packets = 0;
        total_bytes = 0;

        current_frame_num = -1;
        current_subframe_num = -1;

    }

    /** Extracts the frames from the pcap data file buffer
     * \param[in] data - pcap data
     * \param[in] size
     */
    void ExcaliburFrameSimulatorPlugin::extract_frames(const u_char *data, const int &size) {

        Excalibur::AsicCounterBitDepth depth = Excalibur::bitDepth12;

        LOG4CXX_DEBUG(logger_, "Extracting frame(s) from packet");

        int subframe_ctr = int(
                (unsigned char) (data[3]) << 24 | (unsigned char) (data[2]) << 16 | (unsigned char) (data[1]) << 8 |
                (unsigned char) (data[0]));
        int pkt_ctr = int(
                (unsigned char) (data[7]) << 24 | (unsigned char) (data[6]) << 16 | (unsigned char) (data[5]) << 8 |
                (unsigned char) (data[4]));

        bool is_SOF = pkt_ctr & Excalibur::start_of_frame_mask;
        bool is_EOF = pkt_ctr & Excalibur::end_of_frame_mask;

        if (is_SOF) {

            LOG4CXX_DEBUG(logger_, "SOF marker for " + boost::lexical_cast<std::string>(subframe_ctr) + " at packet " +
                                   boost::lexical_cast<std::string>(total_packets));

            // Increment the current subframe index, modulo the number of subframes expected
            current_subframe_num = (current_subframe_num + 1) % Excalibur::num_subframes[depth];

            LOG4CXX_DEBUG(logger_, "Current sub frame " + boost::lexical_cast<std::string>(current_subframe_num));

            if (current_subframe_num == 0) {

                current_frame_num += 1;
                UDPFrame frame(current_frame_num);
                frames.push_back(frame);

            }

            frames[frames.size() - 1].SOF_markers.push_back(subframe_ctr);

        }

        if (is_EOF) {

            long long trailer_frame_ctr = (long long) ((unsigned char) (data[size - 5]) << 24 |
                                                       (unsigned char) (data[size - 6]) << 16 |
                                                       (unsigned char) (data[size - 7]) << 8 |
                                                       (unsigned char) (data[size - 8]));

            LOG4CXX_DEBUG(logger_,
                          "EOF marker for subframe " + boost::lexical_cast<std::string>(subframe_ctr) + " at packet " +
                          boost::lexical_cast<std::string>(total_packets) + \
            " with trailer frame number " + boost::lexical_cast<std::string>(trailer_frame_ctr));

            frames[frames.size() - 1].EOF_markers.push_back(subframe_ctr);
            frames[frames.size() - 1].trailer_frame_num = trailer_frame_ctr;

        }

        boost::shared_ptr<Packet> pkt(new Packet());
        unsigned char *datacp = new unsigned char[size];
        memcpy(datacp, data, size);
        pkt->data = datacp;
        pkt->size = size;
        frames[frames.size() - 1].packets.push_back(pkt);

        total_packets++;

    }

    /** Creates a number of frames
     *
     * @param num_frames - number of frames to create
     */
    void ExcaliburFrameSimulatorPlugin::create_frames(const int &num_frames) {

      Excalibur::AsicCounterBitDepth depth = Excalibur::bitDepth12;

      LOG4CXX_DEBUG(logger_, "Creating frame(s)");

      const int num_packets = Excalibur::num_primary_packets[depth] + Excalibur::num_tail_packets;

      int trailer_frame_number = 1;

      for(int frame=0; frame < num_frames; frame++) {

        for(int sub_frame=0; sub_frame < Excalibur::num_subframes[depth]; sub_frame++) {

          for(int packet=0; packet < num_packets; packet++) {

            bool is_first_primary_packet = ((total_packets) % 66 == 0);
            bool is_tail_packet = ((total_packets + 1) % 66 == 0);

            size_t packet_size = !is_tail_packet ?
                    Excalibur::primary_packet_size + 8 : Excalibur::tail_packet_size[depth] + 8;

            unsigned char *data = new unsigned char[packet_size];

            unsigned int subframe_number = int(total_packets / 66);
            unsigned int packet_number = total_packets + 1;

            data[0] = subframe_number & 0xff;
            data[1] = (subframe_number >> 8) & 0xff;
            data[2] = (subframe_number >> 16) & 0xff;
            data[3] = (subframe_number >> 24) & 0xff;

            if(is_first_primary_packet) {

              data[4] = Excalibur::start_of_frame_mask & 0xff;
              data[5] = (Excalibur::start_of_frame_mask >> 8) & 0xff;
              data[6] = (Excalibur::start_of_frame_mask >> 16) & 0xff;
              data[7] = (Excalibur::start_of_frame_mask >> 24) & 0xff;

            }

            else if(is_tail_packet) {

              data[4] = Excalibur::end_of_frame_mask & 0xff;
              data[5] = (Excalibur::end_of_frame_mask >> 8) & 0xff;
              data[6] = (Excalibur::end_of_frame_mask >> 16) & 0xff;
              data[7] = (Excalibur::end_of_frame_mask >> 24) & 0xff;

              data[packet_size - 8] = trailer_frame_number & 0xff;
              data[packet_size - 7] = (trailer_frame_number >> 8) & 0xff;
              data[packet_size - 6] = (trailer_frame_number >> 16) & 0xff;
              data[packet_size - 5] = (trailer_frame_number >> 24) & 0xff;

            }

            this->extract_frames(data, packet_size);

          }

        }

      }

    }

   /**
    * Get the plugin major version number.
    *
    * \return major version number as an integer
    */
    int ExcaliburFrameSimulatorPlugin::get_version_major() {
        return ODIN_DATA_VERSION_MAJOR;
    }

    /**
     * Get the plugin minor version number.
     *
     * \return minor version number as an integer
     */
    int ExcaliburFrameSimulatorPlugin::get_version_minor() {
        return ODIN_DATA_VERSION_MINOR;
    }

    /**
     * Get the plugin patch version number.
     *
     * \return patch version number as an integer
     */
    int ExcaliburFrameSimulatorPlugin::get_version_patch() {
        return ODIN_DATA_VERSION_PATCH;
    }

    /**
     * Get the plugin short version (e.g. x.y.z) string.
     *
     * \return short version as a string
     */
    std::string ExcaliburFrameSimulatorPlugin::get_version_short() {
        return ODIN_DATA_VERSION_STR_SHORT;
    }

    /**
     * Get the plugin long version (e.g. x.y.z-qualifier) string.
     *
     * \return long version as a string
     */
    std::string ExcaliburFrameSimulatorPlugin::get_version_long() {
        return ODIN_DATA_VERSION_STR;
    }

}
