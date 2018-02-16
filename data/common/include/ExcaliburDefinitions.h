/*
 * ExcaliburDefinitions.h
 *
 *  Created on: Jan 16th, 2017
 *      Author: Tim Nicholls, STFC Application Engineering Group
 */

#ifndef INCLUDE_EXACLIBURDEFINITIONS_H_
#define INCLUDE_EXACLIBURDEFINITIONS_H_

namespace Excalibur {

    static const size_t num_bit_depths = 4;
    typedef enum {
      bitDepthUnknown = -1,
      bitDepth1  = 0,
      bitDepth6  = 1,
      bitDepth12 = 2,
      bitDepth24 = 3
    } AsicCounterBitDepth;

    static const size_t primary_packet_size    = 8000;
    static const size_t num_primary_packets    = 65;
    static const size_t tail_packet_size       = 4296;
    static const size_t num_tail_packets       = 1;
    static const size_t num_subframes          = 2;
    static const size_t max_num_fems           = 6;

    static const uint32_t start_of_frame_mask = 1 << 31;
    static const uint32_t end_of_frame_mask   = 1 << 30;
    static const uint32_t packet_number_mask   = 0x3FFFFFFF;

    static const uint32_t default_frame_number = -1;

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
      uint8_t  packet_state[num_subframes][num_primary_packets + num_tail_packets];
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

    static const size_t subframe_size         = (num_primary_packets * primary_packet_size)
                                              + (num_tail_packets * tail_packet_size);
    static const size_t fem_frame_size        = subframe_size * num_subframes;
    static const size_t max_frame_size        = (fem_frame_size * max_num_fems) + sizeof(FrameHeader);
    static const size_t num_fem_frame_packets = num_subframes * (num_primary_packets + num_tail_packets);


}

#endif /* INCLUDE_EXACLIBURDEFINITIONS_H_ */
