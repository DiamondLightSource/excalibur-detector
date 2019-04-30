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

    /** ExcaliburFrame class
     *
     * An excalibur frame and its packets; defines the SOF and EOF markers
     * for reading an excalibur frame from a pcap file
     */

    class ExcaliburFrame {

        friend class ExcaliburFrameSimulatorPlugin;

    public:

        ExcaliburFrame(const int& frameNum);

    private:

        PacketList packets;

        int frame_number;
        int trailer_frame_num;
        std::vector<int> SOF_markers;
        std::vector<int> EOF_markers;

    };

}

#endif //FRAMESIMULATOR_EXCALIBURFRAME_H
