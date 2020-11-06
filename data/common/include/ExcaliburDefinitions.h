/*
 * ExcaliburDefinitions.h
 *
 *  Created on: Jan 16th, 2017
 *      Author: Tim Nicholls, STFC Application Engineering Group
 */

#ifndef INCLUDE_EXACLIBURDEFINITIONS_H_
#define INCLUDE_EXACLIBURDEFINITIONS_H_

namespace Excalibur {

    static const size_t num_bit_depths = 5;
    typedef enum {
      bitDepthUnknown = -1,
      bitDepth1       = 0,
      bitDepth6       = 1,
      bitDepth12      = 2,
      bitDepth24      = 3,
      bitDepthDual12  = 4,
    } AsicCounterBitDepth;

    static const size_t primary_packet_size    = 8000;
    static const size_t num_primary_packets[num_bit_depths] = { 4, 32, 65, 65, 65 };
    static const size_t max_primary_packets = 65;
    static const size_t tail_packet_size[num_bit_depths] = { 768, 6152, 4296, 4296, 4296 };
    static const size_t num_tail_packets = 1;

    static const size_t num_subframes[num_bit_depths] = {2, 2, 2, 4, 4};
    static const size_t max_num_subframes = 4;

    static const size_t max_num_fems = 6;

    static const uint32_t start_of_frame_mask = 1 << 31;
    static const uint32_t end_of_frame_mask   = 1 << 30;
    static const uint32_t packet_number_mask   = 0x3FFFFFFF;

    static const int32_t default_frame_number = -1;

    typedef struct
    {
    	uint32_t subframe_counter;
    	uint32_t packet_number_flags;
    } PacketHeader;

    typedef struct
    {
    	uint64_t frame_number;
    } SubframeTrailer;

    typedef struct
    {
      uint32_t packets_received;
      uint8_t  sof_marker_count;
      uint8_t  eof_marker_count;
      uint8_t  packet_state[max_num_subframes][max_primary_packets + num_tail_packets];
    } FemReceiveState;

    typedef struct
    {
        uint32_t frame_number;
        uint32_t frame_state;
        struct timespec frame_start_time;
        uint32_t total_packets_received;
        uint8_t total_sof_marker_count;
        uint8_t total_eof_marker_count;
        uint8_t num_active_fems;
        uint8_t active_fem_idx[max_num_fems];
        FemReceiveState fem_rx_state[max_num_fems];
    } FrameHeader;


    inline const std::size_t subframe_size(const AsicCounterBitDepth bit_depth)
    {
      std::size_t subframe_size = (primary_packet_size * num_primary_packets[bit_depth]) +
          (tail_packet_size[bit_depth] * num_tail_packets);
      return subframe_size;
    }

    inline const std::size_t max_frame_size(void)
    {
      std::size_t max_frame_size = sizeof(FrameHeader) +
          (subframe_size(bitDepth24) * num_subframes[bitDepth24] * max_num_fems);
      return max_frame_size;
    }

    inline const std::size_t num_fem_frame_packets(const AsicCounterBitDepth bit_depth)
    {
      std::size_t num_fem_frame_packets = num_subframes[bit_depth] *
          (num_primary_packets[bit_depth] + num_tail_packets);
      return num_fem_frame_packets;
    }

    //! Parse the ASIC counter bit depth configuration string.
    //!
    //! This method parses a configuration string specifying the EXCALIBUR ASIC counter bit
    //! depth currently in use, returning an enumerated constant for use in the decoder.
    //!
    //! \param[in] bit_depth_str - string of the bit depth
    //! \return enumerated constant defining bit depth, or unknown if the string is not recognised
    //!
    inline const AsicCounterBitDepth parse_bit_depth(const std::string bit_depth_str)
    {
      //! Set the default bit depth to return to unknown
      AsicCounterBitDepth bit_depth = bitDepthUnknown;

      // Initialise a mapping of string to bit depth
      static std::map<std::string, Excalibur::AsicCounterBitDepth>bit_depth_map;
      if (bit_depth_map.empty())
      {
        bit_depth_map["1-bit"] = bitDepth1;
        bit_depth_map["6-bit"] = bitDepth6;
        bit_depth_map["12-bit"] = bitDepth12;
        bit_depth_map["24-bit"] = bitDepth24;
        bit_depth_map["dual12-bit"] = bitDepthDual12;
      }

      // Set the bit depth value if present in the map
      if (bit_depth_map.count(bit_depth_str))
      {
        bit_depth = bit_depth_map[bit_depth_str];
      }

      return bit_depth;
    }

}

#endif /* INCLUDE_EXACLIBURDEFINITIONS_H_ */
