#ifndef ORBSLAMPYTHON_H
#define ORBSLAMPYTHON_H

#include <memory>
#include <Python.h>
#include <boost/python.hpp>
#include <ORB_SLAM2/System.h>
#include <ORB_SLAM2/Tracking.h>
#include <ORB_SLAM2/Osmap.h>

class ORBSlamPython
{
public:
    ORBSlamPython(std::string vocabFile, std::string settingsFile,
        ORB_SLAM2::System::eSensor sensorMode = ORB_SLAM2::System::eSensor::RGBD);
    ORBSlamPython(const char* vocabFile, const char* settingsFile,
        ORB_SLAM2::System::eSensor sensorMode = ORB_SLAM2::System::eSensor::RGBD);
    ~ORBSlamPython();
    
    bool initialize();
    bool isRunning();
    bool loadAndProcessMono(std::string imageFile, double timestamp);
    bool processMono(cv::Mat image, double timestamp);
    bool loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp);
    bool processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp);
    bool loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp);
    bool processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp);
    void reset();
    void shutdown();
    ORB_SLAM2::Tracking::eTrackingState getTrackingState() const;
    unsigned int getNumFeatures() const;
    unsigned int getNumMatches() const;
    boost::python::list getKeyframePoints() const;
    boost::python::list getTrajectoryPoints() const;
    boost::python::list getTrackedMappoints() const;
    bool saveSettings(boost::python::dict settings) const;
    boost::python::dict loadSettings() const;
    void setMode(ORB_SLAM2::System::eSensor mode);
    void setRGBMode(bool rgb);
    void setUseViewer(bool useViewer);
    
	void tum_example(boost::python::list arguments);
    static bool saveSettingsFile(boost::python::dict settings, std::string settingsFilename);
    static boost::python::dict loadSettingsFile(std::string settingsFilename);
    
    std::string vocabluaryFile;
    std::string settingsFile;
    ORB_SLAM2::System::eSensor sensorMode;
    std::shared_ptr<ORB_SLAM2::System> system;
    bool bUseViewer;
    bool bUseRGB;
};

class OsmapPython{
	public:
		OsmapPython(ORBSlamPython & _os2python);
		void map_save(std::string basefilename, bool pauseThreads = true);
		void map_load(std::string yamlFilename, bool noSetBad = false, bool pauseThreads = true);
		std::shared_ptr<ORB_SLAM2::Osmap> osmap;
		void verbose_on();	
};


#endif // ORBSLAMPYTHON_H
