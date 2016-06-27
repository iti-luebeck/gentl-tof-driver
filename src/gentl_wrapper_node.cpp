#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gentl_tof_driver/BaslerCameraInfo.h>

/*
    This sample illustrates how to grab and process images using the
    GenTL transport layer interface.
    The camera is configured using GenApi.
    The GenTL v1.5 is supported to demonstrate multi-part buffer handling.

    The standard documents for GenTL and GenApi can be downloaded from here:
    http://www.emva.org/cms/upload/Standards/GenICam_Downloads/GenICam_GenTL_1_5.pdf
    http://www.emva.org/cms/upload/Standards/GenICam_Downloads/GenICam_Standard_v2_0.pdf
*/

#include <string>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <vector>

#if defined (_MSC_VER) && defined (_WIN32)
// We always want to link against the release version of GenApi.
// The following define msut be set before including the GenApi.h header file.
#  define GENICAM_USER_ALWAYS_LINK_RELEASE 1
#endif

#include <TLI/GenTL_v1_5.h>       // GenTL transport layer interface
#include <GenApi/GenApi.h>          // Node map support
#include <PFNC.h>                   // constants for PIXELFORMAT_NAMESPACE_PFNC_32BIT

#include <gentl_tof_driver/GtlSystemModule.h>
#include <gentl_tof_driver/Port.h>


#if defined (_MSC_VER) && defined (_WIN32)
// Link the import libraries for the GenICam GenApi used for configuring the camera device.
// Please note, that it is required to delay load these DLLs. Refer to the project settings
// to see how to instruct the linker to delay load DLLs. ("Properties->Linker->Input->Delay Loaded Dlls"
// resp. /DELAYLOAD linker option).
#  pragma comment (lib, LIB_NAME( "GCBase"))
#  pragma comment (lib, LIB_NAME( "GenApi"))
#endif

using namespace std;
using namespace GenTL;

static const int cCountOfImagesToGrab = 5;
static const int cBufferCount = 2;

ros::Publisher cloud_pub_;
ros::Publisher intensity_pub_;
ros::Publisher confidence_pub_;
ros::Publisher camera_info_pub_;
std::string frame_id_;
std::string device_id_;
int device_id_int_;


// ---------------------------------------------------------------------------
/// \brief This function loads a TL
///
/// \param [in] sysModule Reference to the system module
/// \param [in] libname   File name of the GenTL producer library
///
/// \return success: Handle to the TL, failure: GENTL_INVALID_HANDLE
// ---------------------------------------------------------------------------
TL_HANDLE SelectTL(GenTlConsumer::CGtlSystemModule& sysModule, const char* libname)
{
    GC_ERROR status;

    // Load dynamic library
    if (!sysModule.Init(libname))
        return GENTL_INVALID_HANDLE;

    cout << "Use GenTL Producer '" << libname << "'" << endl;

    TL_HANDLE hTl = GENTL_INVALID_HANDLE;
    status = sysModule.TLOpen(&hTl);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("TLOpen failed:"); return GENTL_INVALID_HANDLE; }

    INFO_DATATYPE iType = INFO_DATATYPE_UNKNOWN;
    char szBuffer[1024];
    size_t iSize = sizeof(szBuffer);
    status = sysModule.TLGetInfo(hTl, TL_INFO_VENDOR, &iType, szBuffer, &iSize);
    if (status == GC_ERR_SUCCESS)
        cout << "  VendorName:    " << szBuffer << "\n";

    iType = INFO_DATATYPE_UNKNOWN;
    iSize = sizeof(szBuffer);
    status = sysModule.TLGetInfo(hTl, TL_INFO_MODEL, &iType, szBuffer, &iSize);
    if (status == GC_ERR_SUCCESS)
        cout << "  ModelName:     " << szBuffer << "\n";

    return hTl;
}

// ---------------------------------------------------------------------------
/// \brief Select the first interface of the given TL
///
/// \param [in] sysModule Reference to the system module
/// \param [in] hTl       Handle to the TL
///
/// \return Interface handle, GENTL_INVALID_HANDLE in case of failure
// ---------------------------------------------------------------------------
IF_HANDLE SelectInterface(const GenTlConsumer::CGtlSystemModule& sysModule, TL_HANDLE hTl)
{
    GC_ERROR status;

    cout << endl;
    cout << "Open first Interface" << endl;

    bool8_t bChanged = false;
    status = sysModule.TLUpdateInterfaceList(hTl, &bChanged, 100);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("TLUpdateInterfaceList failed:"); return GENTL_INVALID_HANDLE; }

    uint32_t iNumInterfaces = 0;
    status = sysModule.TLGetNumInterfaces(hTl, &iNumInterfaces);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("TLGetNumInterfaces failed:"); return GENTL_INVALID_HANDLE; }
    cout << "  NumInterfaces: " << iNumInterfaces << endl;

    if (iNumInterfaces < 1)
    {
        cerr << "Error:\tThis TL returns no interfaces\n";
        return GENTL_INVALID_HANDLE;
    }

    const uint32_t iInterfaceIndex = 0;
    char sIfaceID[1024];
    size_t iSize = sizeof(sIfaceID);
    status = sysModule.TLGetInterfaceID(hTl, iInterfaceIndex, sIfaceID, &iSize);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("TLGetInterfaceID failed:"); return GENTL_INVALID_HANDLE; }
    cout << "  InterfaceID:   " << sIfaceID << endl;

    IF_HANDLE hInterface = GENTL_INVALID_HANDLE;
    status = sysModule.TLOpenInterface(hTl, sIfaceID, &hInterface);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("TLOpenInterface failed:"); return GENTL_INVALID_HANDLE; }

    return hInterface;
}

// ---------------------------------------------------------------------------
/// \brief Open the first device on the given interface
///
/// \param [in] sysModule  Reference to the system module
/// \param [in] hInterface Handle to the interface
///
/// \return Device handle, GENTL_INVALID_HANDLE in case of failure
// ---------------------------------------------------------------------------
DEV_HANDLE SelectDevice(const GenTlConsumer::CGtlSystemModule& sysModule, IF_HANDLE hInterface)
{
  GC_ERROR status;

    bool HasChanged = false;
    status = sysModule.IFUpdateDeviceList(hInterface, &HasChanged, 500);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("IFUpdateDeviceList failed:"); return GENTL_INVALID_HANDLE; }

    uint32_t iNumDevs = 0;
    status = sysModule.IFGetNumDevices(hInterface, &iNumDevs);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("IFGetNumDevices failed:"); return GENTL_INVALID_HANDLE; }
    cout << "  NumDevices:    " << iNumDevs << endl;

    if (iNumDevs == 0)
    {
        cerr << "Error:\tNo device found\n";
        return GENTL_INVALID_HANDLE;
    }

    char szName[1024];
    size_t iSize = sizeof(szName);

    if (device_id_ == "first")
    {
        
        cout << endl;
        cout << "Open first Device" << endl;

        const uint32_t iDeviceIndex = 0;
        status = sysModule.IFGetDeviceID(hInterface, iDeviceIndex, szName, &iSize);
        if (status != GC_ERR_SUCCESS) { sysModule.HandleError("IFGetDeviceID failed:"); return GENTL_INVALID_HANDLE; }
    } 
    else
    {
        
        std::stringstream ss;
        ss << device_id_int_;
        device_id_ = ss.str();

        cout << endl;
        cout << "Open device with serial " << device_id_ << endl;

        for(int i = 0; i<iNumDevs; i++)
        {
            char szNameTemp[1024];
            size_t iSizeTemp = sizeof(szName);
            status = sysModule.IFGetDeviceID(hInterface, i, szNameTemp, &iSizeTemp);
            if (status != GC_ERR_SUCCESS) { sysModule.HandleError("IFGetDeviceID failed:"); return GENTL_INVALID_HANDLE; }
            
            INFO_DATATYPE iTypeTemp = INFO_DATATYPE_UNKNOWN;
            char szBufferTemp[1024];
            iSizeTemp = sizeof(szBufferTemp);
            status = sysModule.IFGetDeviceInfo(hInterface, szNameTemp, DEVICE_INFO_DISPLAYNAME, &iTypeTemp, szBufferTemp, &iSizeTemp);
            if (status != GC_ERR_SUCCESS) { sysModule.HandleError("IFGetDeviceInfo failed:"); return GENTL_INVALID_HANDLE; }

            std::string serial = szBufferTemp;
            serial = serial.substr(7,8);

            if(device_id_ == serial)
            {
                strcpy(szName, szNameTemp);
                break;
            }
            
            if(i==iNumDevs-1)
            {
                cerr << "Error:\tNo device found\n";
                return GENTL_INVALID_HANDLE;
            }
        }
    }

    cout << "  DeviceID:      " << szName << endl;

    INFO_DATATYPE iType = INFO_DATATYPE_UNKNOWN;
    char szBuffer[1024];
    iSize = sizeof(szBuffer);
    status = sysModule.IFGetDeviceInfo(hInterface, szName, DEVICE_INFO_DISPLAYNAME, &iType, szBuffer, &iSize);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("IFGetDeviceInfo failed:"); return GENTL_INVALID_HANDLE; }
    cout << "  Display name:  " << szBuffer << endl;

    DEV_HANDLE hDev = GENTL_INVALID_HANDLE;
    status = sysModule.IFOpenDevice(hInterface, szName, DEVICE_ACCESS_EXCLUSIVE, &hDev);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("IFOpenDevice failed:"); return GENTL_INVALID_HANDLE; }

    return hDev;
}

// ---------------------------------------------------------------------------
/// \brief Retrieve the GenICam xml from a given port
///
/// \param [in] sysModule Reference to the system module
/// \param [in] hPort     Port to read from
///
/// \return string containing the XML
// ---------------------------------------------------------------------------
GenICam::gcstring RetrieveXML(const GenTlConsumer::CGtlSystemModule& sysModule, PORT_HANDLE hPort)
{
    GenICam::gcstring gcstrXml;
    GC_ERROR status;

    // retrieve URL
    uint32_t iNumURLs = 0;
    status = sysModule.GCGetNumPortURLs(hPort, &iNumURLs);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("GCGetNumPortURLs failed:"); return gcstrXml; }
    if (iNumURLs == 0)
    {
        cerr << "Error:\tGCGetNumPortURLs returned zero\n";
        return gcstrXml;
    }

    char sURL[2048];
    size_t iUrlLength = sizeof(sURL);
    const uint32_t iURLIndex = 0;  // support only first URL here
    INFO_DATATYPE iType = INFO_DATATYPE_UNKNOWN;

    status = sysModule.GCGetPortURLInfo(hPort, iURLIndex, URL_INFO_URL, &iType, sURL, &iUrlLength);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("GCGetPortURLInfo failed:"); return gcstrXml; }
    if (iUrlLength >= sizeof(sURL))
        return gcstrXml;

    // Read XML Address
    std::string strXMLAddress(sURL);
    std::string strLocation = strXMLAddress.substr(0, 6);
    std::transform(strLocation.begin(), strLocation.end(), strLocation.begin(), (int(*)(int)) toupper);

    if (strLocation == "LOCAL:")
    {
        std::string fullFilename;
        uint64_t iAddr = 0;
        size_t iXMLSize = 0;
        if (!sysModule.SplitLocalUrl(strXMLAddress.substr(6), fullFilename, iAddr, iXMLSize))
        {
            cerr << "Error getting address or length from URL " << strXMLAddress.substr(6) << endl;
            return gcstrXml;
        }

        std::vector<char> buf(iXMLSize + 1);  // incl. zero termination

        status = sysModule.GCReadPort(hPort, iAddr, &buf[0], &iXMLSize);
        if (status != GC_ERR_SUCCESS) { sysModule.HandleError("GCReadPort failed:"); return gcstrXml; }

        // ensure zero termination
        buf[iXMLSize] = '\0';
        gcstrXml = &buf[0];
    }

    return gcstrXml;
}

// ---------------------------------------------------------------------------
/// \brief Load xml from Client::port and instantiate the nodemap
///
/// \param [in] sysModule Reference to the system module
/// \param [in] hPort     Retrieve the nodemap from the provided port implementation
///
/// \return Nodemap, NULL in case of failure
///
/// can throw
// ---------------------------------------------------------------------------
GenApi::CNodeMapRef *GetNodeMap(const GenTlConsumer::CGtlSystemModule& sysModule, PORT_HANDLE hPort)
{
    GC_ERROR status;
    GenApi::CNodeMapRef *pDeviceMap = new GenApi::CNodeMapRef;
    GenICam::gcstring strXML = RetrieveXML(sysModule, hPort);

    if (strXML.empty()) throw runtime_error("Unable to retrieve XML");

    pDeviceMap->_LoadXMLFromString(strXML);

    CPort *pPortImpl = new CPort(sysModule, hPort);
    GenApi::IPort *pGenApiPort = dynamic_cast<GenApi::IPort *>(pPortImpl);

    char szPortName[256];
    size_t iSize = sizeof(szPortName);
    INFO_DATATYPE iType = INFO_DATATYPE_UNKNOWN;
    status = sysModule.GCGetPortInfo(hPort, PORT_INFO_ID, &iType, szPortName, &iSize);
    if (status != GC_ERR_SUCCESS) sysModule.HandleError("GCGetPortInfo failed:");

    cout << "  PortID:        " << szPortName << endl;

    GenICam::gcstring gcstrPortName = "Device"/*szPortName*/;
    bool bResult = pDeviceMap->_Connect(pGenApiPort, gcstrPortName);
    if (!bResult) cerr << "Error: NodeMap connect failed" << endl;

    return pDeviceMap;
}

// T is one of the described types from INFO_DATATYPE_LIST with fixed size, so except string types.
template <typename T>
bool GetBufferInfo(const GenTlConsumer::CGtlSystemModule& sysModule, DS_HANDLE hDatastream, BUFFER_HANDLE hBuffer, BUFFER_INFO_CMD iInfoCmd, T* value)
{
    INFO_DATATYPE iType = INFO_DATATYPE_UNKNOWN;
    size_t iSize = sizeof(T);   // fixed size is used
    const GC_ERROR status = sysModule.DSGetBufferInfo(hDatastream, hBuffer, iInfoCmd, &iType, value, &iSize);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSGetBufferInfo failed:"); return false; }
    return true;
}

// T is one of the described types from INFO_DATATYPE_LIST with fixed size, so except string types.
template <typename T>
bool GetBufferPartInfo(const GenTlConsumer::CGtlSystemModule& sysModule, DS_HANDLE hDatastream, BUFFER_HANDLE hBuffer, uint32_t iPartIndex, BUFFER_PART_INFO_CMD iInfoCmd, T* value)
{
    INFO_DATATYPE iType = INFO_DATATYPE_UNKNOWN;
    size_t iSize = sizeof(T);   // fixed size is used
    const GC_ERROR status = sysModule.DSGetBufferPartInfo(hDatastream, hBuffer, iPartIndex, iInfoCmd, &iType, value, &iSize);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSGetBufferPartInfo failed:"); return false; }
    return true;
}

// ---------------------------------------------------------------------------
/// \brief Provide default camera information
///
/// \param [in] width   width of the camera image
/// \param [in] height  height of the camera image
/// \param [in] f       focal length of the camera lense
///
/// \return Camera information (width, height, D, distortion_model, K, R, P)
// ---------------------------------------------------------------------------
sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f)
{
    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

    info->width  = width;
    info->height = height;

    // No distortion
    info->D.resize(5, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = info->K[4] = f;
    info->K[2] = (width / 2) - 0.5;

    // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
    // This formula keeps the principal point the same in VGA and SXGA modes
    info->K[5] = (width * (3./8.)) - 0.5;
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0]  = info->P[5] = f; // fx, fy
    info->P[2]  = info->K[2];     // cx
    info->P[6]  = info->K[5];     // cy
    info->P[10] = 1.0;

    return info;
}

// ---------------------------------------------------------------------------
/// \brief Retrieve multi-part buffer information and display it on standard output
///
/// \param [in] sysModule   Reference to the system module
/// \param [in] hDatastream Data stream handle
/// \param [in] hBuffer     Buffer handle
///
/// \return true on success, false in case of failure
// ---------------------------------------------------------------------------
bool RetrieveBufferPart(const GenTlConsumer::CGtlSystemModule& sysModule, DS_HANDLE hDatastream, BUFFER_HANDLE hBuffer, uint32_t partIndex, ros::Time acquisition_time, uint64_t buffer_seq, BaslerCameraInfo &basler_camera_info)
{
    void* baseAddr = 0;
    size_t size = 0;
    size_t width = 0;
    size_t height = 0;
    size_t dataType = 0;        // one of PARTDATATYPE_IDS
    uint64_t dataFormat = 0;    // depends on dataFormatNamespace:
                                // for PIXELFORMAT_NAMESPACE_PFNC_32BIT(4) the constants from PFNC.h will be used

    GetBufferPartInfo(sysModule, hDatastream, hBuffer, partIndex, BUFFER_PART_INFO_BASE, &baseAddr);
    GetBufferPartInfo(sysModule, hDatastream, hBuffer, partIndex, BUFFER_PART_INFO_DATA_SIZE, &size);
    GetBufferPartInfo(sysModule, hDatastream, hBuffer, partIndex, BUFFER_PART_INFO_WIDTH, &width);
    GetBufferPartInfo(sysModule, hDatastream, hBuffer, partIndex, BUFFER_PART_INFO_HEIGHT, &height);
    GetBufferPartInfo(sysModule, hDatastream, hBuffer, partIndex, BUFFER_PART_INFO_DATA_FORMAT, &dataFormat);

    switch ( partIndex )
    {
    case 0: // range
        if ( dataFormat == PFNC_Coord3D_ABC32f )
        {
            // The part has the expected data format.
            // Access the data.
#pragma pack(push, 1)
            struct Coord3D
            {
                float x;
                float y;
                float z;
            } *pCoord = static_cast<Coord3D*>(baseAddr); // + height / 2 * width + width / 2;
#pragma pack(pop)

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cloud->header.frame_id = frame_id_;
            cloud->header.stamp = pcl_conversions::toPCL(acquisition_time);
            cloud->header.seq = buffer_seq;
            cloud->width = width;
            cloud->height = height;
            cloud->points.resize(width * height);

            for (int i = 0; i < width * height; i++) {
                pcl::PointXYZ &p = cloud->points[i];
                if (pCoord->z < 10000.f) {
                    p.x = 0.001f * pCoord->x;
                    p.y = 0.001f * pCoord->y;
                    p.z = 0.001f * pCoord->z;
                } else {
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                }
                pCoord++;
            }

            cloud_pub_.publish(cloud);
        } else {
            cout << "Unexpected format for the first image part" << endl;
        }
        break;

    case 1: // intensity
        if ( dataFormat == PFNC_Mono16 )
        {
            // The part has the expected data format.
            // Access the data.
            uint16_t *pIntensity = static_cast<uint16_t*>(baseAddr);

            cv::Mat intensity_img(height, width, CV_16U);
            intensity_img.setTo(0);
            for (size_t i = 0; i < height; i++) {
                for (size_t j = 0; j < width; j++) {
                    intensity_img.at<unsigned short>(i,j) = (unsigned short) *pIntensity;
                    pIntensity++;
                }
            }
            cv_bridge::CvImage cv_ptr;
            /*cv_ptr.encoding = sensor_msgs::image_encodings::MONO16;
            cv_ptr.header.seq = 0;
            cv_ptr.image = intensity_img;*/
            intensity_img.convertTo(intensity_img, CV_8U, 0.00390625);
            cv::equalizeHist(intensity_img,intensity_img);
            cv::imshow("image",intensity_img);
            cv::waitKey(10);
            cv_ptr.encoding = sensor_msgs::image_encodings::MONO8;
            cv_ptr.header.seq = 0;
            cv_ptr.image = intensity_img;

            sensor_msgs::Image::Ptr intensity_msg = cv_ptr.toImageMsg();
            intensity_msg->header.frame_id = frame_id_;
            intensity_msg->header.stamp = acquisition_time;
            intensity_msg->header.seq = buffer_seq;
            intensity_pub_.publish(intensity_msg);

            sensor_msgs::CameraInfoPtr info_msg;
            double HFOW = 57.0 * ( M_PI / 180.0 ); // from http://www.baslerweb.com/media/documents/BAS1508_ToF_EN__web.pdf
            double f = width / ( 2.0 * tan( HFOW / 2.0 ) );
            info_msg = basler_camera_info.getCameraInfo(width,height,f);//getDefaultCameraInfo(width, height, f);
            info_msg->header.stamp    = acquisition_time;
            info_msg->header.frame_id = frame_id_;
            camera_info_pub_.publish(info_msg);
        }
        else
        {
            cout << "Unexpected format for the second image part" << endl;
        }

        break;

    case 2: // confidence
        if ( dataFormat == PFNC_Mono16 )
        {
            // The part has the expected data format.
            // Access the data.
            uint16_t *pConfidence = static_cast<uint16_t*>(baseAddr);

            cv::Mat confidence_img(height, width, CV_16U);
            confidence_img.setTo(0);
            for (size_t i = 0; i < height; i++) {
                for (size_t j = 0; j < width; j++) {
                    confidence_img.at<unsigned short>(i,j) = (unsigned short) *pConfidence;
                    pConfidence++;
                }
            }

            cv_bridge::CvImage cv_ptr;
            cv_ptr.encoding = sensor_msgs::image_encodings::MONO16;
            cv_ptr.header.seq = 0;
            cv_ptr.image = confidence_img;
            sensor_msgs::Image::Ptr confidence_msg = cv_ptr.toImageMsg();
            confidence_msg->header.frame_id = frame_id_;
            confidence_msg->header.stamp = acquisition_time;
            confidence_msg->header.seq = buffer_seq;
            confidence_pub_.publish(confidence_msg);
        }
        else
        {
            cout << "Unexpected format for the third image part" << endl;
        }
        break;
    }


    return true;
}


// ---------------------------------------------------------------------------
/// \brief Retrieve buffer information and display it on standard output
///
/// \param [in] sysModule   Reference to the system module
/// \param [in] hDatastream Data stream handle
/// \param [in] hBuffer     Buffer handle
///
/// \return true on success, false in case of failure
// ---------------------------------------------------------------------------
bool RetrieveResult(const GenTlConsumer::CGtlSystemModule& sysModule, DS_HANDLE hDatastream, BUFFER_HANDLE hBuffer, BaslerCameraInfo basler_camera_info)
{
    size_t payloadType = PAYLOAD_TYPE_UNKNOWN;  // one of PAYLOADTYPE_INFO_IDS
    uint64_t frameID = 0;
    bool8_t bufferIsIncomplete = false;

    if (!GetBufferInfo(sysModule, hDatastream, hBuffer, BUFFER_INFO_PAYLOADTYPE, &payloadType))
        return false;

    GetBufferInfo(sysModule, hDatastream, hBuffer, BUFFER_INFO_FRAMEID, &frameID);
    GetBufferInfo(sysModule, hDatastream, hBuffer, BUFFER_INFO_IS_INCOMPLETE, &bufferIsIncomplete);

    uint64_t buffer_time = 0;
    GetBufferInfo(sysModule, hDatastream, hBuffer, BUFFER_INFO_TIMESTAMP, &buffer_time);
//    std::cout << std::setprecision(16) << "buffer time: " << buffer_time << std::endl;
    auto acquisition_time = ros::Time::now();

    if (payloadType == PAYLOAD_TYPE_MULTI_PART)
    {
        uint32_t iNumParts = 0;
        const GC_ERROR status = sysModule.DSGetNumBufferParts(hDatastream, hBuffer, &iNumParts);
        if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSGetNumBufferParts failed:"); return false; }
        if (iNumParts == 0)
        {
            cerr << "Error:\tDSGetNumBufferParts() returned zero parts\n";
            return false;
        }

        // Process the individual buffer parts
        for (uint32_t partIndex = 0; partIndex < iNumParts; ++partIndex) {
            RetrieveBufferPart(sysModule, hDatastream, hBuffer, partIndex, acquisition_time, frameID, basler_camera_info);
        }
    }
    else
    {
        // Unexpected. From a ToF camera we always retrieve multi part images.
    }

    return true;
}

// ---------------------------------------------------------------------------
/// \brief Configure the components of the multi-part images the camera
/// is going to send.
///
/// \param [in] pDeviceMap The camera's node map used for configuring the camera
///
/// \return true on success, false in case of failure
// ---------------------------------------------------------------------------
void ParametrizeCamera(GenApi::CNodeMapRef *pDeviceMap)
{
    // For this sample, we set up the camera to send image buffers consisting of 3 Parts:
    // - Depth information (range)
    // - The grey value image (intensity)
    // - The confidence image

    GenApi::CEnumerationPtr ptrImageComponentSelector = pDeviceMap->_GetNode("ImageComponentSelector");
    ptrImageComponentSelector->FromString("Range");
    GenApi::CBooleanPtr ptrImageComponentEnable = pDeviceMap->_GetNode("ImageComponentEnable");
    ptrImageComponentEnable->SetValue(true);

    // Range information can be send either as a 16 Bit grey value image, for this sample we want to acquire 3D coordinates
    // Note: For changing the format of an image component, the Component Selector must be set to the component
    // to configure first (see above).
    GenApi::CEnumerationPtr ptrPixelFormat = pDeviceMap->_GetNode("PixelFormat");
    ptrPixelFormat->FromString("Coord3D_ABC32f");  // x-y-z coordinates, floats
    // For using 16 bit integer depth information instead of 3D coordinates:
    // ptrPixelFormat->FromString("Mono16");

    GenApi::CEnumerationPtr ptrimageComponentSelector = pDeviceMap->_GetNode("ImageComponentSelector");
    ptrimageComponentSelector->FromString("Intensity");
    GenApi::CBooleanPtr ptrimageComponentEnable = pDeviceMap->_GetNode("ImageComponentEnable");
    ptrimageComponentEnable->SetValue(true);

    GenApi::CEnumerationPtr ptrImageComponentSelector_ = pDeviceMap->_GetNode("ImageComponentSelector");
    ptrImageComponentSelector_->FromString("Confidence");
    GenApi::CBooleanPtr ptrImageComponentEnable_ = pDeviceMap->_GetNode("ImageComponentEnable");
    ptrImageComponentEnable_->SetValue(true);

}

// ---------------------------------------------------------------------------
/// \brief Inquire image dimensions, allocate memory, setup acquisition, acquire, stop and free resources
///
/// \param [in] sysModule Reference to the system module
/// \param [in] hDev      Device handle
///
/// \return true on success, false in case of failure
// ---------------------------------------------------------------------------
bool Grab(const GenTlConsumer::CGtlSystemModule& sysModule, const DEV_HANDLE hDev, BaslerCameraInfo &basler_camera_info)
{
    bool result = true;
    GC_ERROR status;

    cout << endl;
    cout << "Setup Acquisition" << endl;

    // retrieve access to device
    PORT_HANDLE hRemoteDevicePort = NULL;
    status = sysModule.DevGetPort(hDev, &hRemoteDevicePort);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DevGetPort failed:"); return false; }

    // open data stream
    uint32_t iNumStreams = 0;
    status = sysModule.DevGetNumDataStreams(hDev, &iNumStreams);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DevGetNumDataStreams failed:"); return false; }
    cout << "  NumDataStreams:" << iNumStreams << endl;

    if (iNumStreams == 0)
    {
        cerr << "Error:\tNo data stream found\n";
        return false;
    }

    const uint32_t iDataStreamIndex = 0;
    char sDataStreamID[512];
    size_t iSize = sizeof(sDataStreamID);
    status = sysModule.DevGetDataStreamID(hDev, iDataStreamIndex, &sDataStreamID[0], &iSize);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DevGetDataStreamID failed:"); return false; }
    cout << "  DataStreamID:  " << &sDataStreamID[0] << endl;

    DS_HANDLE hDatastream = NULL;
    status = sysModule.DevOpenDataStream(hDev, &sDataStreamID[0], &hDatastream);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DevOpenDataStream failed:"); return false; }

    uint32_t uiWidth = 0;
    uint32_t uiHeight = 0;
    uint32_t uiPayloadSize = 0;

    GenApi::CNodeMapRef *pDeviceMap = NULL;
    try
    {
        // Create a so called node map used for accessing camera parameters.
        pDeviceMap = GetNodeMap(sysModule, hRemoteDevicePort);

        // Parametrize the camera
        ParametrizeCamera( pDeviceMap);


        // Query some camera properties.
        GenApi::CIntegerPtr ptrWidth = pDeviceMap->_GetNode("Width");
        GenApi::CIntegerPtr ptrHeight = pDeviceMap->_GetNode("Height");
        GenApi::CIntegerPtr ptrPayloadSize = pDeviceMap->_GetNode("PayloadSize");

        uiWidth = static_cast<uint32_t>(ptrWidth->GetValue());
        uiHeight = static_cast<uint32_t>(ptrHeight->GetValue());
        uiPayloadSize = static_cast<uint32_t>(ptrPayloadSize->GetValue());
    }
    catch (GenICam::GenericException &E)
    {
        cerr << "Error " << E.GetDescription() << endl;
        return false;
    }
    cout << "Values reported by the camera:" << endl;
    cout << "  Image width:                     " << uiWidth << endl;
    cout << "  Image height:                    " << uiHeight << endl;
    cout << "  PayloadSize of one image buffer: " << uiPayloadSize << endl;

    INFO_DATATYPE iType = INFO_DATATYPE_UNKNOWN;
    size_t iPayloadSize;
    iSize = sizeof(iPayloadSize);

    // Query the data stream about the payload size of images.
    sysModule.DSGetInfo(hDatastream, STREAM_INFO_PAYLOAD_SIZE, &iType, &iPayloadSize, &iSize);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("Error in DSGetInfo: "); return false; }
    cout << "  PayloadSize:   " << iPayloadSize << endl;

    // We use PayloadSize from the camera node map (since the stream's payload information has to be fixed)
    const size_t iImageSize = uiPayloadSize;

    // Allocate image buffers and register them with the stream object.
    std::vector<std::vector<char> > ImageBuffer(cBufferCount, std::vector<char>(iImageSize));
    BUFFER_HANDLE phBuffer = NULL;
    for (int i = 0; i < cBufferCount; i++)
    {
        // Before a buffer can be used for grabbing, it must be announced.
        status = sysModule.DSAnnounceBuffer(hDatastream, &ImageBuffer[i][0], iImageSize, reinterpret_cast<void *>(i), &phBuffer);
        if (status != GC_ERR_SUCCESS) { sysModule.HandleError("Error in DSAnnounceBuffer: "); return false; }

        // Pass the buffer to the stream object to be filled with data.
        status = sysModule.DSQueueBuffer(hDatastream, phBuffer);
        if (status != GC_ERR_SUCCESS) { sysModule.HandleError("Error in DSQueueBuffer: "); return false; }
    }

    // Register New Buffer Event. The event will be signaled if an buffer has been acquired.
    EVENT_HANDLE pEventNewBuffer = NULL;
    status = sysModule.GCRegisterEvent(hDatastream, EVENT_NEW_BUFFER, &pEventNewBuffer);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("Register Event failed:"); return false; }

    // Start Acquisition. Both, the stream and the camera must be started-
    status = sysModule.DSStartAcquisition(hDatastream, ACQ_START_FLAGS_DEFAULT, GENTL_INFINITE);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSStartAcquisition failed:"); return false; }
    GenApi::CCommandPtr ptrStartAcq = pDeviceMap->_GetNode("AcquisitionStart");
    ptrStartAcq->Execute();  // The camera now acquires images.

    // Acquisition Loop
    int ImageCount = cCountOfImagesToGrab;
    bool bRun = true;
    while (ros::ok())
    {
        EVENT_NEW_BUFFER_DATA NewImageEventData;
        size_t iSize = sizeof(NewImageEventData);
        status = sysModule.EventGetData(pEventNewBuffer, &NewImageEventData, &iSize, 500);  // wait up to 500 ms for a buffer

        if (status == GC_ERR_TIMEOUT)
        {
            cout << "Timeout" << endl;
        }
        else if (status != GC_ERR_SUCCESS)
        {
            sysModule.HandleError("EventGetData failed:");
            bRun = false;
            result = false;
            continue;
        }
        else
        {
            // Retrieve and process the image data
            RetrieveResult(sysModule, hDatastream, NewImageEventData.BufferHandle, basler_camera_info);

            // When processing of the image is done, pass it back to the stream to be filled
            // with new data again.
            status = sysModule.DSQueueBuffer(hDatastream, NewImageEventData.BufferHandle);
            if (status != GC_ERR_SUCCESS)
            {
                sysModule.HandleError("Error in DSQueueBuffer: ");
                bRun = false;
                result = false;
                continue;
            }
        }
    }


    // Stop Acquisition. Both, the stream and the camera must be stopped.
    status = sysModule.DSStopAcquisition(hDatastream, ACQ_STOP_FLAGS_DEFAULT);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSStopAcquisition failed:"); return false; }
        GenApi::CCommandPtr ptrStopAcq = pDeviceMap->_GetNode("AcquisitionStop");
    ptrStopAcq->Execute();

    // Cleanup
    delete pDeviceMap;

    status = sysModule.DSFlushQueue(hDatastream, ACQ_QUEUE_INPUT_TO_OUTPUT);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSFlushQueue failed:"); return false; }
    status = sysModule.DSFlushQueue(hDatastream, ACQ_QUEUE_OUTPUT_DISCARD);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSFlushQueue failed:"); return false; }

    status = sysModule.DSClose(hDatastream);
    if (status != GC_ERR_SUCCESS) { sysModule.HandleError("DSClose failed:"); return false; }

    return result;
}

// ---------------------------------------------------------------------------
/// \brief Main
///
/// \param [in] argc
/// \param [in] argv[]
/// \return
// ---------------------------------------------------------------------------
#ifndef _WIN32
#  define _TCHAR char
#endif

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gentl_wrapper_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string strPathToCti;
    pn.param("cti_path", strPathToCti, std::string("/opt/BaslerToF/lib64/gentlproducer/gtl/ProducerTOF.cti"));

    pn.param("frame_id", frame_id_, std::string("camera_link"));
    pn.getParam("device_id", device_id_);
    pn.getParam("device_id", device_id_int_);

    cloud_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("cloud", 1);
    intensity_pub_ = n.advertise<sensor_msgs::Image>("intensity/image_raw", 1);
    confidence_pub_ = n.advertise<sensor_msgs::Image>("confidence/image_raw", 1);
    camera_info_pub_ = n.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    GenTlConsumer::CGtlSystemModule sysModule;

    BaslerCameraInfo camera_info;

    TL_HANDLE hTl = SelectTL(sysModule, strPathToCti.c_str());
    if (hTl != GENTL_INVALID_HANDLE)
    {
        IF_HANDLE hInterface = SelectInterface(sysModule, hTl);
        if (hInterface != GENTL_INVALID_HANDLE)
        {
            DEV_HANDLE hDevice = SelectDevice(sysModule, hInterface);
            if (hDevice != GENTL_INVALID_HANDLE)
            {
                Grab(sysModule, hDevice, camera_info);

                sysModule.DevClose(hDevice);
            }
            sysModule.IFClose(hInterface);
        }
        sysModule.TLClose(hTl);
    }

    return 0;
}
