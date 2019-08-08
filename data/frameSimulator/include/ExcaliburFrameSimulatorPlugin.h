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
#include "FrameSimulatorPluginUDP.h"

namespace FrameSimulator {

    /** ExcaliburFrameSimulatorPlugin
     *
     *  'prepare_packets' (and then 'extract_frames') is called on setup if a pcap file is specified: this takes the
     *  content of the pcap file and organises it in frames to store
     *  'create_frames' is called on setup if no pcap file is specified
     *  'replay_frames' is called by simulate: this will replay the created/stored frames
     */
    class ExcaliburFrameSimulatorPlugin : public FrameSimulatorPluginUDP {

    public:

        ExcaliburFrameSimulatorPlugin();

        virtual int get_version_major();
        virtual int get_version_minor();
        virtual int get_version_patch();
        virtual std::string get_version_short();
        virtual std::string get_version_long();

    protected:

        virtual void extract_frames(const u_char* data, const int& size);
        virtual void create_frames(const int &num_frames);

    private:

        /** Pointer to logger **/
        LoggerPtr logger_;

    };

    REGISTER(FrameSimulatorPlugin, ExcaliburFrameSimulatorPlugin, "ExcaliburFrameSimulatorPlugin");

}

#endif //EXCALIBUR_DETECTOR_EXCALIBURFRAMESIMULATORPLUGIN_H
