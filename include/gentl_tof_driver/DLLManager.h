/*
    Manage platform-independent loading of shared libraries.
*/

#pragma once

#ifndef DLL_MANAGER_H_INCLUDED__
#define DLL_MANAGER_H_INCLUDED__

#include <string>

#if defined (_MSC_VER) && defined (_WIN32)
#   include <windows.h>
#elif defined(__GNUC__) && defined(__linux__)
#   include <dlfcn.h>
#else
#   error Unsupported platform
#endif


class CDLLManager
{
public:
    ///////////////////////////////////////////////////////////////////////
    CDLLManager(void)
        : m_hDLL(NULL)
    {
    }

    ///////////////////////////////////////////////////////////////////////
    ~CDLLManager(void)
    {
        unload();
    }

public:

    ///////////////////////////////////////////////////////////////////////
    bool load(const char* pName)
    {
#if defined (_MSC_VER) && defined (_WIN32)

        HMODULE h = ::LoadLibraryA(pName);

        if (h == NULL)
            setErrorMsg(::GetLastError());
        else
            m_error = "";

#elif defined(__GNUC__) && defined(__linux__)

        void* h = dlopen(pName, RTLD_NOW | RTLD_LOCAL);

        const char* error = dlerror();
        m_error = error ? error : "";
#endif

        if (h == NULL)
        {
            // error loading DLL
            return false;
        }

        // after we successfully loaded the DLL
        // unload prev lib and set the member
        unload();

        m_hDLL = h;

        return true;
    }


    ///////////////////////////////////////////////////////////////////////
    bool unload()
    {
        if (isValid())
        {
            // m_error only has to be set when returning false on error.
#if defined (_MSC_VER) && defined (_WIN32)
            ::FreeLibrary(m_hDLL);
#elif defined(__GNUC__) && defined(__linux__)
            dlclose(m_hDLL);
#endif
            m_hDLL = NULL;
            return true;
        }
        else
        {
            return false;
        }
    }


    ///////////////////////////////////////////////////////////////////////
    void* getFunctionPointer(const char* pszName)
    {
        if (!isValid())
        {
            return NULL;
        }

#if defined (_MSC_VER) && defined (_WIN32)
        void* pFunc(::GetProcAddress(m_hDLL, pszName));

        if (pFunc == NULL)
            setErrorMsg(::GetLastError());
        else
            m_error = "";

#elif defined(__GNUC__) && defined(__linux__)
        dlerror();  // Clear dlerror() to distinguish between "symbol not found" and an error after dlsym()

        void* pFunc;
        *(void**)(&pFunc) = dlsym(m_hDLL, pszName);

        const char* error = dlerror();
        m_error = error ? error : "";
#endif

        return pFunc;
    }


    ///////////////////////////////////////////////////////////////////////
    bool isValid() const
    {
        bool res(m_hDLL != NULL);
        return res;
    }


    ///////////////////////////////////////////////////////////////////////
    const char* getLastErrorMsg() const
    {
        return m_error.c_str();
    }


#if defined (_MSC_VER) && defined (_WIN32)
private:
    void setErrorMsg(DWORD errorCode)
    {
        LPSTR pMsgBuf = NULL;
        if (::FormatMessageA(
            FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL,
            errorCode,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPSTR)&pMsgBuf,
            0, NULL))
        {
            m_error = pMsgBuf;
            LocalFree(pMsgBuf);
        }
        else
        {
            m_error = "FormatMessage failed for error code 0x";
            char buffer[sizeof(long) * 2 + 1];
            m_error += _ltoa(errorCode, buffer, 16);
        }
    }
#endif

private:
#if defined (_MSC_VER) && defined (_WIN32)
    HMODULE m_hDLL;
#elif defined(__GNUC__) && defined(__linux__)
    void* m_hDLL;
#endif
    std::string m_error;
};


#endif //#ifndef DLL_MANAGER_H_INCLUDED__
