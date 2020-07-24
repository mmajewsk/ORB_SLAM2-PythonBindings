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
    boost::python::list getKeyframeList() const;
    boost::python::list getTrajectoryPoints() const;
    boost::python::list getTrackedMappoints() const;
    bool saveSettings(boost::python::dict settinplegs) const;
    boost::python::dict loadSettings() const;
    cv::Mat current_pose;
    void setMode(ORB_SLAM2::System::eSensor mode);
    void setRGBMode(bool rgb);
    void setUseViewer(bool useViewer);
    
    static bool saveSettingsFile(boost::python::dict settings, std::string settingsFilename);
    static boost::python::dict loadSettingsFile(std::string settingsFilename);
    
    std::string vocabluaryFile;
    std::string settingsFile;
    ORB_SLAM2::System::eSensor sensorMode;
    std::shared_ptr<ORB_SLAM2::System> system;
    bool bUseViewer;
    bool bUseRGB;
	std::shared_ptr<ORB_SLAM2::Osmap> sys_osmap;
	void osmap_init();
	void map_save(std::string basefilename, bool pauseThreads);
	void map_load(std::string yamlFilename, bool noSetBad, bool pauseThreads );
	void activate_localisation_only();
	void deactivate_localisation_only();
    cv::Mat get_current_pose();
    std::string ORBSlamPython::get_current_pose_txt();


};


#endif // ORBSLAMPYTHON_H
