#ifndef FRAMESIMULATOR_EXCALIBURFRAMESIMULATORPLUGIN_H
#define FRAMESIMULATOR_EXCALIBURFRAMESIMULATORPLUGIN_H

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/helpers/exception.h>
using namespace log4cxx;
using namespace log4cxx::helpers;

#include <boost/shared_ptr.hpp>
#include <map>
#include <string>
#include <stdexcept>

#include "ClassLoader.h"
#include "ExcaliburFrame.h"
#include "pcapFrameSimulatorPlugin.h"

#include<arpa/inet.h>
#include<net/ethernet.h>
#include<netinet/udp.h>
#include<netinet/ip.h>

namespace FrameSimulator {

    typedef std::vector<ExcaliburFrame> ExcaliburFrames;

    class ExcaliburFrameSimulatorPlugin : public pcapFrameSimulatorPlugin {

    public:

        ExcaliburFrameSimulatorPlugin();

        virtual int get_version_major();
        virtual int get_version_minor();
        virtual int get_version_patch();
        virtual std::string get_version_short();
        virtual std::string get_version_long();

    protected:

        virtual void prepare_packets(const struct pcap_pkthdr *header, const u_char *buffer);
        virtual void replay_packets();

    private:

        void extract_frames(const u_char* buffer, const int& header_len);//, int size);

        /** Pointer to logger **/
        LoggerPtr logger_;

        int total_packets;
        int total_bytes;

        int current_frame_num;
        int current_subframe_num;

        ExcaliburFrames frames;

        struct iphdr *iph;
        struct udphdr *udph;

    };

    REGISTER(FrameSimulatorPlugin, ExcaliburFrameSimulatorPlugin, "ExcaliburFrameSimulatorPlugin");

}

#endif //EXCALIBUR_DETECTOR_EXCALIBURFRAMESIMULATORPLUGIN_H
