/*
    GenApi IPort Implementation to be used by a GenTL Consumer.
*/

#pragma once

#ifndef GC_PORT_H_
#define GC_PORT_H_

#include <GenApi/GenApi.h>
#include <TLI/GenTL_v1_5.h>
#include "GtlSystemModule.h"
#include <stdexcept>

class CPort: public GenApi::IPort
{
public:
    CPort(const GenTlConsumer::CGtlSystemModule& sysModule, GenTL::PORT_HANDLE hPort)
      : m_sysModule(sysModule)
      , m_hPort(hPort)
    { }

    virtual ~CPort(void)
    { }

    // Reads a chunk of bytes from the port
    virtual void Read(void *pBuffer, int64_t Address, int64_t Length)
    {
        size_t iSize = static_cast<size_t>(Length);
        GenTL::GC_ERROR status = m_sysModule.GCReadPort(m_hPort, Address, pBuffer, &iSize);
        if ((status != GenTL::GC_ERR_SUCCESS) || (static_cast<int64_t>(iSize) != Length))
            throw std::runtime_error("Unable to Read Register");
    }

    // Writes a chunk of bytes to the port
    virtual void Write(const void *pBuffer, int64_t Address, int64_t Length)
    {
        size_t iSize = static_cast<size_t>(Length);
        GenTL::GC_ERROR status = m_sysModule.GCWritePort(m_hPort, Address, pBuffer, &iSize);
        if ((status != GenTL::GC_ERR_SUCCESS) || (static_cast<int64_t>(iSize) != Length))
            throw std::runtime_error("Unable to Write Register");
    }

    virtual GenApi::EAccessMode GetAccessMode(void) const
    {
        return GenApi::RW;
    }

private:
    const GenTlConsumer::CGtlSystemModule& m_sysModule;
    GenTL::PORT_HANDLE m_hPort;
};

#endif //GC_PORT_H_
