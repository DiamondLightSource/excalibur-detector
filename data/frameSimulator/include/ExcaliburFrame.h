#ifndef FRAMESIMULATOR_EXCALIBURFRAME_H
#define FRAMESIMULATOR_EXCALIBURFRAME_H

#include <vector>
#include <iostream>
#include <sstream>
#include <string>

#include "Packet.h"

namespace FrameSimulator {

    class ExcaliburFrameSimulatorPlugin;

    typedef std::vector<Packet> PacketList;

    class ExcaliburFrame {

        friend class ExcaliburFrameSimulatorPlugin;

    public:

        ExcaliburFrame(const int& frameNum);

        static const unsigned int SOF_marker = 1 << 31;
        static const unsigned int EOF_marker = 1 << 30;
        static const int num_subframes = 2;

    private:

        PacketList packets;

        int frame_number;
        int trailer_frame_num;
        std::vector<int> SOF_markers;
        std::vector<int> EOF_markers;

    };



}

#endif //FRAMESIMULATOR_EXCALIBURFRAME_H
