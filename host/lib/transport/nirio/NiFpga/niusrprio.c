/*
 * Copyright (c) 2012,
 * National Instruments Corporation.
 * All rights reserved.
 */

#include "niusrprio.h"

/*
 * Platform specific includes.
 */
#if NiFpga_Windows
   #include <windows.h>
#elif NiFpga_VxWorks
   #include <vxWorks.h>
   #include <symLib.h>
   #include <loadLib.h>
   #include <sysSymTbl.h>
   MODULE_ID VxLoadLibraryFromPath(const char* path, int flags);
   STATUS VxFreeLibrary(MODULE_ID library, int flags);
#elif NiFpga_Linux
   #include <stdlib.h>
   #include <stdio.h>
   #include <dlfcn.h>
#else
   #error
#endif

/*
 * Platform specific defines.
 */
#if NiFpga_Windows
   #define niusrprio_CCall   __cdecl
   #define niusrprio_StdCall __stdcall
#else
   #define niusrprio_CCall
   #define niusrprio_StdCall
#endif

/*
 * Global library handle, or NULL if the library isn't loaded.
 */
#if NiFpga_Windows
   static HMODULE niusrprio_library = NULL;
#elif NiFpga_VxWorks
   static MODULE_ID niusrprio_library = NULL;
#elif NiFpga_Linux
   static void* niusrprio_library = NULL;
#else
   #error
#endif

static int32_t (niusrprio_CCall *niusrprio_open_)(
        uint32_t const deviceIdentifier, uint64_t * const handle) = NULL;

int32_t niusrprio_open(
        uint32_t const deviceIdentifier, uint64_t * const handle)
{
   return niusrprio_open_
        ? niusrprio_open_(deviceIdentifier, handle)
        : NiFpga_Status_ResourceNotInitialized;
}

static int32_t (niusrprio_CCall *niusrprio_close_)(
        uint64_t const handle) = NULL;

int32_t niusrprio_close(
        uint64_t const handle)
{
   return niusrprio_close_
        ? niusrprio_close_(handle)
        : NiFpga_Status_ResourceNotInitialized;
}

static int32_t (niusrprio_CCall *niusrprio_getFlashAutoLoadMode_)(
        uint64_t const handle, uint32_t * const autoLoadMode) = NULL;

int32_t niusrprio_getFlashAutoLoadMode(
        uint64_t const handle, uint32_t * const autoLoadMode)
{
   return niusrprio_getFlashAutoLoadMode_
        ? niusrprio_getFlashAutoLoadMode_(handle, autoLoadMode)
        : NiFpga_Status_ResourceNotInitialized;
}

static int32_t (niusrprio_CCall *niusrprio_setFlashAutoLoadMode_)(
        uint64_t const handle, uint32_t const autoLoadMode) = NULL;

int32_t niusrprio_setFlashAutoLoadMode(
        uint64_t const handle, uint32_t const autoLoadMode)
{
   return niusrprio_setFlashAutoLoadMode_
        ? niusrprio_setFlashAutoLoadMode_(handle, autoLoadMode)
        : NiFpga_Status_ResourceNotInitialized;
}

static int32_t (niusrprio_CCall *niusrprio_downloadToFlash_)(
        uint64_t const handle, uint8_t const * const bitstream, uint32_t const bitstreamSize) = NULL;

int32_t niusrprio_downloadToFlash(
        uint64_t const handle, uint8_t const * const bitstream, uint32_t const bitstreamSize)
{
   return niusrprio_downloadToFlash_
        ? niusrprio_downloadToFlash_(handle, bitstream, bitstreamSize)
        : NiFpga_Status_ResourceNotInitialized;
}

static int32_t (niusrprio_CCall *niusrprio_getNumberOfDevices_)(
        uint64_t * const numberOfDevices) = NULL;

int32_t niusrprio_getNumberOfDevices(
        uint64_t * const numberOfDevices)
{
   return niusrprio_getNumberOfDevices_
        ? niusrprio_getNumberOfDevices_(numberOfDevices)
        : NiFpga_Status_ResourceNotInitialized;
}

static int32_t (niusrprio_CCall *niusrprio_getDevicesInformation_)(
        uint64_t const numberOfDevices, uint32_t * const deviceIdentifiers, uint64_t * const serialNumbers) = NULL;

int32_t niusrprio_getDevicesInformation(
        uint64_t const numberOfDevices, uint32_t * const deviceIdentifiers, uint64_t * const serialNumbers)
{
   return niusrprio_getDevicesInformation_
        ? niusrprio_getDevicesInformation_(numberOfDevices, deviceIdentifiers, serialNumbers)
        : NiFpga_Status_ResourceNotInitialized;
}


/**
 * A NULL-terminated array of all entry point functions.
 */
static const struct
{
   const char* const name;
   void** const address;
} niusrprio_functions[] =
{
   {"niusrprio_open",   (void**)(void*)&niusrprio_open_},
   {"niusrprio_close",  (void**)(void*)&niusrprio_close_},
   {"niusrprio_getFlashAutoLoadMode",  (void**)(void*)&niusrprio_getFlashAutoLoadMode_},
   {"niusrprio_setFlashAutoLoadMode",  (void**)(void*)&niusrprio_setFlashAutoLoadMode_},
   {"niusrprio_downloadToFlash",  (void**)(void*)&niusrprio_downloadToFlash_},
   {"niusrprio_getNumberOfDevices",  (void**)(void*)&niusrprio_getNumberOfDevices_},
   {"niusrprio_getDevicesInformation",  (void**)(void*)&niusrprio_getDevicesInformation_},
   {NULL, NULL}
};

int32_t niusrprio_Initialize(void)
{
   /* if the library isn't already loaded */
   if (!niusrprio_library)
   {
      int i;
      /* load the library */
      #if NiFpga_Windows
         niusrprio_library = LoadLibraryA("niusrprio.dll");
      #elif NiFpga_VxWorks
         niusrprio_library = VxLoadLibraryFromPath("niusrprio.out", 0);
      #elif NiFpga_Linux
         const char* const library = "libniusrprio.so";
         niusrprio_library = dlopen(library, RTLD_LAZY);
         if (!niusrprio_library)
            fprintf(stderr, "Error opening %s: %s\n", library, dlerror());
      #else
         #error
      #endif
      if (!niusrprio_library)
         return NiFpga_Status_ResourceNotFound;
      /* get each exported function */
      for (i = 0; niusrprio_functions[i].name; i++)
      {
         const char* const name = niusrprio_functions[i].name;
         void** const address = niusrprio_functions[i].address;
         #if NiFpga_Windows
            *address = GetProcAddress(niusrprio_library, name);
            if (!*address)
               return NiFpga_Status_VersionMismatch;
         #elif NiFpga_VxWorks
            SYM_TYPE type;
            if (symFindByName(sysSymTbl,
                              (char*)name,
                              (char**)address,
                              &type) != OK)
               return NiFpga_Status_VersionMismatch;
         #elif NiFpga_Linux
            *address = dlsym(niusrprio_library, name);
            if (!*address)
               return NiFpga_Status_VersionMismatch;
         #else
            #error
         #endif
      }
   }
   return NiFpga_Status_Success;
}

int32_t niusrprio_Finalize(void)
{
   /* if the library is currently loaded */
   if (niusrprio_library)
   {
      int i;
      NiFpga_Status status = NiFpga_Status_Success;
      /* unload the library */
      #if NiFpga_Windows
         if (!FreeLibrary(niusrprio_library))
            status = NiFpga_Status_ResourceNotInitialized;
      #elif NiFpga_VxWorks
         if (VxFreeLibrary(niusrprio_library, 0) != OK)
            status = NiFpga_Status_ResourceNotInitialized;
      #elif NiFpga_Linux
         if (dlclose(niusrprio_library))
            status = NiFpga_Status_ResourceNotInitialized;
      #else
         #error
      #endif
      /* null out the library and each exported function */
      niusrprio_library = NULL;
      for (i = 0; niusrprio_functions[i].name; i++)
         *niusrprio_functions[i].address = NULL;
      return status;
   }
   else
      return NiFpga_Status_ResourceNotInitialized;
}
