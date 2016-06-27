/*
    GenTL System module class implementation
*/

#include <sstream>
#include <iostream>

#include <gentl_tof_driver/GtlSystemModule.h>

// Initializes a function pointer. The function may be optionally.
#define INIT_OPT_FCT_PTR( _PTR ) \
    _PTR = (GenTL::P##_PTR) m_Module.getFunctionPointer( #_PTR );

// Initializes a function pointer. The pointer is checked for a NULL ptr.
#define INIT_FCT_PTR( _PTR ) \
    INIT_OPT_FCT_PTR( _PTR ); \
    if ( NULL == _PTR ) { res = false; std::cerr << "Invalid GenTL DLL. Function " #_PTR " is missing.\n"; }



using namespace GenTL;

namespace GenTlConsumer
{

    /** \brief Represents the GenTL System module

    Provides access to all functions exported by a GenTL producer.

    */
    CGtlSystemModule::CGtlSystemModule()
        : GCInitLib(NULL)
        , GCCloseLib(NULL)
        , GCGetInfo(NULL)
        , GCGetLastError(NULL)
        , GCReadPort(NULL)
        , GCWritePort(NULL)
        , GCReadPortStacked(NULL)
        , GCWritePortStacked(NULL)
        , GCGetPortURL(NULL)
        , GCGetPortInfo(NULL)
        , GCGetNumPortURLs(NULL)
        , GCGetPortURLInfo(NULL)
        , GCRegisterEvent(NULL)
        , GCUnregisterEvent(NULL)
        , TLClose(NULL)
        , TLGetInfo(NULL)
        , TLGetNumInterfaces(NULL)
        , TLGetInterfaceID(NULL)
        , TLGetInterfaceInfo(NULL)
        , TLOpenInterface(NULL)
        , TLUpdateInterfaceList(NULL)
        , IFClose(NULL)
        , IFGetInfo(NULL)
        , IFGetNumDevices (NULL)
        , IFGetDeviceID(NULL)
        , IFUpdateDeviceList(NULL)
        , IFGetDeviceInfo(NULL)
        , IFOpenDevice(NULL)
        , IFGetParentTL(NULL)
        , DevGetPort(NULL)
        , DevGetNumDataStreams(NULL)
        , DevGetDataStreamID(NULL)
        , DevOpenDataStream(NULL)
        , DevGetInfo(NULL)
        , DevClose(NULL)
        , DevGetParentIF(NULL)
        , DSAnnounceBuffer(NULL)
        , DSAllocAndAnnounceBuffer(NULL)
        , DSFlushQueue(NULL)
        , DSStartAcquisition(NULL)
        , DSStopAcquisition(NULL)
        , DSGetInfo(NULL)
        , DSGetBufferID(NULL)
        , DSClose(NULL)
        , DSRevokeBuffer(NULL)
        , DSQueueBuffer(NULL)
        , DSGetBufferInfo(NULL)
        , DSGetBufferChunkData(NULL)
        , DSGetParentDev(NULL)
        , DSGetNumBufferParts(NULL)
        , DSGetBufferPartInfo(NULL)
        , EventGetData(NULL)
        , EventGetDataInfo(NULL)
        , EventGetInfo(NULL)
        , EventFlush(NULL)
        , EventKill(NULL)
    {
    }

    CGtlSystemModule::~CGtlSystemModule(void)
    {
        if ( NULL != GCCloseLib )
        {
            GC_ERROR res = GCCloseLib();
            if ( res != GC_ERR_SUCCESS )
            {
                std::cerr << "Failed to call GCCloseLib():" << res << std::endl;
            }
        }
    }


    bool CGtlSystemModule::Init(const char* dllname)
    {
        bool res = true;
        const bool loaded = m_Module.load( dllname );
        if (!loaded)
        {
            std::cerr << "Failed to load module " << dllname << ": " << m_Module.getLastErrorMsg() << std::endl;
            return false;
        }


        INIT_FCT_PTR( GCInitLib );
        INIT_FCT_PTR( GCCloseLib );
        INIT_FCT_PTR( GCGetInfo );
        INIT_FCT_PTR( GCGetLastError );
        INIT_FCT_PTR( GCReadPort );
        INIT_FCT_PTR( GCWritePort );
        INIT_FCT_PTR( GCReadPortStacked );          // GenTL v1.1
        INIT_FCT_PTR( GCWritePortStacked );         // GenTL v1.1
        INIT_FCT_PTR( GCGetPortURL );
        INIT_FCT_PTR( GCGetPortInfo );
        INIT_FCT_PTR( GCGetNumPortURLs );           // GenTL v1.1
        INIT_FCT_PTR( GCGetPortURLInfo );           // GenTL v1.1
        INIT_FCT_PTR( GCRegisterEvent );
        INIT_FCT_PTR( GCUnregisterEvent );

        INIT_FCT_PTR( TLOpen );
        INIT_FCT_PTR( TLClose );
        INIT_FCT_PTR( TLGetInfo );
        INIT_FCT_PTR( TLGetNumInterfaces );
        INIT_FCT_PTR( TLGetInterfaceID );
        INIT_FCT_PTR( TLGetInterfaceInfo );
        INIT_FCT_PTR( TLOpenInterface );
        INIT_FCT_PTR( TLUpdateInterfaceList );

        INIT_FCT_PTR( IFClose );
        INIT_FCT_PTR( IFGetInfo );
        INIT_FCT_PTR( IFGetNumDevices );
        INIT_FCT_PTR( IFGetDeviceID );
        INIT_FCT_PTR( IFUpdateDeviceList );
        INIT_FCT_PTR( IFGetDeviceInfo );
        INIT_FCT_PTR( IFOpenDevice );
        INIT_FCT_PTR( IFGetParentTL );              // GenTL v1.4

        INIT_FCT_PTR( DevGetPort );
        INIT_FCT_PTR( DevGetNumDataStreams );
        INIT_FCT_PTR( DevGetDataStreamID );
        INIT_FCT_PTR( DevOpenDataStream );
        INIT_FCT_PTR( DevGetInfo );
        INIT_FCT_PTR( DevClose );
        INIT_FCT_PTR( DevGetParentIF );             // GenTL v1.4

        INIT_FCT_PTR( DSAnnounceBuffer );
        INIT_FCT_PTR( DSAllocAndAnnounceBuffer );
        INIT_FCT_PTR( DSFlushQueue );
        INIT_FCT_PTR( DSStartAcquisition );
        INIT_FCT_PTR( DSStopAcquisition );
        INIT_FCT_PTR( DSGetInfo );
        INIT_FCT_PTR( DSGetBufferID );
        INIT_FCT_PTR( DSClose );
        INIT_FCT_PTR( DSRevokeBuffer );
        INIT_FCT_PTR( DSQueueBuffer );
        INIT_FCT_PTR( DSGetBufferInfo );
        INIT_FCT_PTR( DSGetBufferChunkData );       // GenTL v1.3
        INIT_FCT_PTR( DSGetParentDev );             // GenTL v1.4
        INIT_FCT_PTR( DSGetNumBufferParts );        // GenTL v1.5
        INIT_FCT_PTR( DSGetBufferPartInfo );        // GenTL v1.5

        INIT_FCT_PTR( EventGetData );
        INIT_FCT_PTR( EventGetDataInfo );
        INIT_FCT_PTR( EventGetInfo );
        INIT_FCT_PTR( EventFlush );
        INIT_FCT_PTR( EventKill );


        GC_ERROR ret = GCInitLib();
        if ( ret != GC_ERR_SUCCESS )
        {
            std::cerr << "Failed to initialize GenTL producer: Error " << ret << std::endl;
            res = false;
        }

        return res;
    }

    bool CGtlSystemModule::SplitLocalUrl(const std::string& strLocal, std::string& fullFilename, uint64_t& address, size_t& length)
    {
        /*
         * Format of strLocal:
         *   "[///]filename.extension;address;length"
         *   Example: tlguru_system_rev1.xml;F0F00000;3BF
         */

        std::stringstream ss;

        // skip optional leading slashes
        std::string slashes("///");
        if (strLocal.find(slashes) == 0)
            ss << strLocal;
        else
            ss << strLocal.substr(slashes.length());

        // get filename
        std::getline(ss, fullFilename, ';');

        // get address;length from remainder
        uint64_t addr = 0;
        size_t len = 0;

        ss >> std::hex >> addr;

        // match semicolon
        if (ss.peek() == ';')
            ss.ignore();
        else
            return false;

        ss >> len;

        if (ss.fail() || !ss.eof())
            return false;

        address = addr;
        length = len;

        return true;
    }

    // ---------------------------------------------------------------------------
    /// \brief Print error string to cerr
    ///
    /// \param [in] pText  Pointer to char buffer to print in addition to the
    ///                    TL error
    // ---------------------------------------------------------------------------
    void CGtlSystemModule::HandleError(const char *pText) const
    {
      GC_ERROR iErrorCode = GC_ERR_NOT_INITIALIZED;
      char sErrorText[4096];
      size_t iSize = sizeof(sErrorText);

      std::cerr << "Error:\t" << pText << std::endl;

      GC_ERROR status = GCGetLastError(&iErrorCode, sErrorText, &iSize);
      if (status == GC_ERR_SUCCESS)
          std::cerr << "\t" << sErrorText << std::endl;
      else
          std::cerr << "\tNo additional information" << std::endl;

      std::cerr << std::endl;
    }
}
