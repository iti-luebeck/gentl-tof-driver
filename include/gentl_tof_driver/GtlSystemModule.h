/*
    GenTL System module class definition
*/

#pragma once

#ifndef _GTL_SYSTEM_MODULE__H_
#define _GTL_SYSTEM_MODULE__H_

#include <string>
#include <TLI/GenTL_v1_5.h>
#include "DLLManager.h"

namespace GenTlConsumer
{

// Represents the GenTL system module
class CGtlSystemModule
{

public:
    CGtlSystemModule();
    ~CGtlSystemModule(void);

    bool Init(const char* dllname);

    // global functions
    GenTL::PGCInitLib                 GCInitLib;
    GenTL::PGCCloseLib                GCCloseLib;
    GenTL::PGCGetInfo                 GCGetInfo;
    GenTL::PGCGetLastError            GCGetLastError;
    GenTL::PGCReadPort                GCReadPort;
    GenTL::PGCWritePort               GCWritePort;
    GenTL::PGCReadPortStacked         GCReadPortStacked;      // GenTL v1.1
    GenTL::PGCWritePortStacked        GCWritePortStacked;     // GenTL v1.1
    GenTL::PGCGetPortURL              GCGetPortURL;           // legacy function
    GenTL::PGCGetPortInfo             GCGetPortInfo;
    GenTL::PGCGetNumPortURLs          GCGetNumPortURLs;       // GenTL v1.1
    GenTL::PGCGetPortURLInfo          GCGetPortURLInfo;       // GenTL v1.1
    GenTL::PGCRegisterEvent           GCRegisterEvent;
    GenTL::PGCUnregisterEvent         GCUnregisterEvent;

    // TL functions
    GenTL::PTLOpen                    TLOpen;
    GenTL::PTLClose                   TLClose;
    GenTL::PTLGetInfo                 TLGetInfo;
    GenTL::PTLGetNumInterfaces        TLGetNumInterfaces;
    GenTL::PTLGetInterfaceID          TLGetInterfaceID;
    GenTL::PTLGetInterfaceInfo        TLGetInterfaceInfo;
    GenTL::PTLOpenInterface           TLOpenInterface;
    GenTL::PTLUpdateInterfaceList     TLUpdateInterfaceList;

    // Interface functions
    GenTL::PIFClose                   IFClose;
    GenTL::PIFGetInfo                 IFGetInfo;
    GenTL::PIFGetNumDevices           IFGetNumDevices;
    GenTL::PIFGetDeviceID             IFGetDeviceID;
    GenTL::PIFUpdateDeviceList        IFUpdateDeviceList;
    GenTL::PIFGetDeviceInfo           IFGetDeviceInfo;
    GenTL::PIFOpenDevice              IFOpenDevice;
    GenTL::PIFGetParentTL             IFGetParentTL;          // GenTL v1.4

    // Device functions
    GenTL::PDevGetPort                DevGetPort;
    GenTL::PDevGetNumDataStreams      DevGetNumDataStreams;
    GenTL::PDevGetDataStreamID        DevGetDataStreamID;
    GenTL::PDevOpenDataStream         DevOpenDataStream;
    GenTL::PDevGetInfo                DevGetInfo;
    GenTL::PDevClose                  DevClose;
    GenTL::PDevGetParentIF            DevGetParentIF;         // GenTL v1.4

    // Data stream functions
    GenTL::PDSAnnounceBuffer          DSAnnounceBuffer;
    GenTL::PDSAllocAndAnnounceBuffer  DSAllocAndAnnounceBuffer;
    GenTL::PDSFlushQueue              DSFlushQueue;
    GenTL::PDSStartAcquisition        DSStartAcquisition;
    GenTL::PDSStopAcquisition         DSStopAcquisition;
    GenTL::PDSGetInfo                 DSGetInfo;
    GenTL::PDSGetBufferID             DSGetBufferID;
    GenTL::PDSClose                   DSClose;
    GenTL::PDSRevokeBuffer            DSRevokeBuffer;
    GenTL::PDSQueueBuffer             DSQueueBuffer;
    GenTL::PDSGetBufferInfo           DSGetBufferInfo;
    GenTL::PDSGetBufferChunkData      DSGetBufferChunkData;   // GenTL v1.3
    GenTL::PDSGetParentDev            DSGetParentDev;         // GenTL v1.4
    GenTL::PDSGetNumBufferParts       DSGetNumBufferParts;    // GenTL v1.5
    GenTL::PDSGetBufferPartInfo       DSGetBufferPartInfo;    // GenTL v1.5

    // Event functions
    GenTL::PEventGetData              EventGetData;
    GenTL::PEventGetDataInfo          EventGetDataInfo;
    GenTL::PEventGetInfo              EventGetInfo;
    GenTL::PEventFlush                EventFlush;
    GenTL::PEventKill                 EventKill;

    // Helper functions
    void HandleError(const char* pText) const;
    static bool SplitLocalUrl(const std::string& strLocal, std::string& fullFilename, uint64_t& address, size_t& length);

private:
    CDLLManager         m_Module;       // handle to the DLL

};

}  // namespace

#endif
