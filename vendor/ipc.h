#ifndef HEKKY_IPC_INC
#define HEKKY_IPC_INC
#include <inttypes.h>

// IPC.h - Single header IPC library for Windows & Linux
// v1.1 - (C) Hekky, under MIT license.
// 
//  Concepts:
//    - Shared / streaming memory :: A block of memory thats shared between programs. This is usually filled with data such that one
//                                   process is continuously writing TO this buffer and other processes simply READ from it.
//    - Operation :: An operation reads / writes to the shared memory buffer. We use the nomenclature of operation to avoid race
//                   conditions, each unique condition has its own mutex and event pair.
//    - Command   :: Effectively a function invocation. We use comamnd IDs to identify commands to jump to the desired callback on the server.
//  
// Usage:
//     #define IPC_IMPLEMENTATION
//     #include "ipc.h"
// 
// Platforms:
//     - Windows
//     - Linux
// 
// Design:
//   - cmds are numeric events. i recommend making an enum for them for better readibility.
//   - data is POD shared via shared memory.
//   - each cmd should have a command associated with it, but isnt necessary.
//   - a streaming buffer is mainainted too, allocated at init time though.
//   - allocate a "stack" for event arguments, and then enough data for your streams. you can use void* to access the data directly.
// 
//   Shared memory layout:
// 
//    o-----------------o
//    |    cmd_type     |
//    o-----------------o
//    |                 |
//    |  Command_XXX_t  |  -> function stack effectively
//    |                 |
//    |-----------------o
//    |                 |
//    |    Streaming    |  -> shared memory for streaming operations
//    |      buffer     |
//    |                 |
//    o-----------------o
// 
// @NOTE: currently relies on C++ - need to make this work in pure C eventually!
// 

#define IPC_COUNT_OF(arr) (sizeof(arr) / sizeof((arr)[0]))

typedef void* IpcHandle_t;
static const IpcHandle_t k_hInvalidIpcHandle = ((IpcHandle_t)(-1));
typedef uint64_t IpcCommandType_t;
typedef void* IpcNativeHandle_t;
typedef void (*pfnIpcFunctionCallback_t)(IpcCommandType_t cmdType, IpcHandle_t hIpcServer, void* pArguments, void* userdata);

struct IpcFunction_t {
    IpcCommandType_t commandType = 0;
    const char* szFunctionName = nullptr;
    pfnIpcFunctionCallback_t callback = nullptr; // required on server, ignored on client
};

struct IpcOperation_t {
    const char* szIdentifier = nullptr; // unique name, passed to OS for names
    size_t dwSharedMemoryOffset = 0; // offset of data to write in shared memory
    IpcNativeHandle_t hNativeMutex = 0; // on windows, HANDLE to mutex
    IpcNativeHandle_t hNativeEvent = 0; // on windows, HANDLE to mutex
};

struct IpcCreateArguments_t {
    const char* szSharedMemoryName = nullptr; // name used for the shared memory
    size_t dwSharedBufferSizeBytes = 0; // size in bytes of the shared memory thats not used for function invocation. function invocation reserves 4KB of data. on windows is bound by DWORD
    const IpcFunction_t* aFunctions = nullptr;
    size_t dwFunctionCount = 0;
    IpcOperation_t* aOperations = nullptr;
    size_t dwOperationCount = 0;
    void* userData = nullptr; // passed to userdata in callbacks
};

IpcHandle_t ipc_server_init(IpcCreateArguments_t args);
IpcHandle_t ipc_client_init(IpcCreateArguments_t args);
bool ipc_server_shutdown(IpcHandle_t* hIpcServer);
bool ipc_client_shutdown(IpcHandle_t* hIpcClient);

/* Invokes a function from the client with id unCmdType with arguments pArgs */
bool ipc_client_dispatch_function(IpcHandle_t hIpcClient, IpcCommandType_t unCmdType, void* pArgs, size_t dwArgsSize);

/* Registers the IpcOperation with the IpcServer. Must be invoked before you can use read/write shared memory. */
bool ipc_server_register_operation(IpcHandle_t hIpcServer, IpcOperation_t* ipcOperation);
/* Writes the given memory into the shared memory at the provided offset. Think of this as a memcpy with extra steps. */
bool ipc_server_write_shared_memory(IpcHandle_t hIpcServer, IpcOperation_t ipcOperation, void* pSrc, size_t destSize);
/* Reads the given memory from the shared memory at the provided offset. Think of this as a memcpy with extra steps. */
bool ipc_client_read_shared_memory(IpcHandle_t hIpcClient, IpcOperation_t ipcOperation, void* pDest, size_t destSize);

#ifdef IPC_IMPLEMENTATION

#if defined(_WIN32)
#define IPC_OS_WIN32 1
#else
#define IPC_OS_WIN32 0
#endif

#if defined(__gnu_linux__) || defined(__linux__)
#define IPC_OS_LINUX 1
#else
#define IPC_OS_LINUX 0
#endif

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
#include <thread>
#include <atomic>

#ifndef IPC_FUNCTION_ARGS_TOTAL_SIZE
// 16KB
#define IPC_FUNCTION_ARGS_TOTAL_SIZE (1024*16)
#endif // IPC_FUNCTION_ARGS_TOTAL_SIZE

#if IPC_OS_WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // WIN32_LEAN_AND_MEAN
#ifndef NOVIRTUALKEYCODES
#define NOVIRTUALKEYCODES
#endif // NOVIRTUALKEYCODES
#ifndef NOSOUND
#define NOSOUND
#endif // NOSOUND
#ifndef NOICONS
#define NOICONS
#endif // NOICONS
#ifndef NOMINMAX
#define NOMINMAX
#endif // NOMINMAX
#include <windows.h>
#elif IPC_OS_LINUX
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <sys/stat.h>
#endif

struct __internal__IpcFunction_t {
    IpcCommandType_t unCmdType = 0;           // the command ID for this function
    pfnIpcFunctionCallback_t pfnCallback = 0; // callback to be invoked by IPC server
};

struct __internal__IpcHandle_t {
    // generally so few that theyd fit in cache so who cares
    std::vector<__internal__IpcFunction_t> registed_functions = {};
    std::vector<IpcOperation_t> registed_operations = {};
    size_t nextFunctionOffset = 0;
    const char* szSharedMemoryName = nullptr;
    IpcNativeHandle_t hSharedMemory = NULL; // on windows, HANDLE to filemapping
    void* pSharedMemory = nullptr; // on windows, void* to the actual memory
    IpcNativeHandle_t hNativeMutex = 0; // on windows, HANDLE to mutex
    IpcNativeHandle_t hNativeEvent = 0; // on windows, HANDLE to mutex
    std::thread poll_thread;
    std::atomic<bool> is_running{ false };
    void* userData = nullptr;
};

#if IPC_OS_LINUX
#define IPC_S_RW_ALL (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
#endif

void __internal_ipc_server_poll_requests(IpcHandle_t hIpcServer) {
    __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)hIpcServer;

#if IPC_OS_WIN32
    while (handleInternal->is_running) {
        DWORD result = WaitForSingleObject((HANDLE)handleInternal->hNativeEvent, 100);

        if (result == WAIT_OBJECT_0) {
            WaitForSingleObject((HANDLE)handleInternal->hNativeMutex, INFINITE);
            IpcCommandType_t* cmdType = (IpcCommandType_t*)handleInternal->pSharedMemory;
            void* pArgs = (void*)((char*)handleInternal->pSharedMemory + sizeof(IpcCommandType_t));

            for (const auto& func : handleInternal->registed_functions) {
                if (func.unCmdType == *cmdType) {
                    if (func.pfnCallback) {
                        func.pfnCallback(*cmdType, hIpcServer, pArgs, handleInternal->userData);
                    }
                    break;
                }
            }

            ResetEvent((HANDLE)handleInternal->hNativeEvent);
            ReleaseMutex((HANDLE)handleInternal->hNativeMutex);
        }
    }
#elif IPC_OS_LINUX
    sem_t* semEvt = (sem_t*)handleInternal->hNativeEvent;
    sem_t* semMtx = (sem_t*)handleInternal->hNativeMutex;

    while (handleInternal->is_running) {
        struct timespec ts;
        if (clock_gettime(CLOCK_REALTIME, &ts) == -1) continue;
        ts.tv_nsec += 100000000; // 100ms
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000;
        }

        int res = sem_timedwait(semEvt, &ts);
        if (res == 0) {
            if (sem_wait(semMtx) == 0) {
                IpcCommandType_t* cmdType = (IpcCommandType_t*)handleInternal->pSharedMemory;
                void* pArgs = (void*)((char*)handleInternal->pSharedMemory + sizeof(IpcCommandType_t));

                for (const auto& func : handleInternal->registed_functions) {
                    if (func.unCmdType == *cmdType) {
                        if (func.pfnCallback) {
                            func.pfnCallback(*cmdType, hIpcServer, pArgs, handleInternal->userData);
                        }
                        break;
                    }
                }

                sem_post(semMtx);
            }
        }
        else if (errno != ETIMEDOUT && errno != EINTR) {
            handleInternal->is_running = false;
        }
    }
#else
#error "Unsupported platform"
#endif
}

IpcHandle_t ipc_server_init(IpcCreateArguments_t args) {
    if (!args.szSharedMemoryName) {
        return k_hInvalidIpcHandle;
    }

    __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)calloc(1, sizeof(__internal__IpcHandle_t));

    if (!handleInternal) {
        return k_hInvalidIpcHandle;
    }

    handleInternal->userData = args.userData;
    handleInternal->szSharedMemoryName = args.szSharedMemoryName;
    size_t totalSharedMemSize = IPC_FUNCTION_ARGS_TOTAL_SIZE + args.dwSharedBufferSizeBytes;

#if IPC_OS_WIN32
    handleInternal->hSharedMemory = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, (DWORD)totalSharedMemSize, args.szSharedMemoryName);
    if (handleInternal->hSharedMemory == NULL) {
        free(handleInternal);
        return k_hInvalidIpcHandle;
    }

    handleInternal->pSharedMemory = MapViewOfFile(handleInternal->hSharedMemory, FILE_MAP_ALL_ACCESS, 0, 0, totalSharedMemSize);
    if (handleInternal->pSharedMemory == NULL) {
        CloseHandle(handleInternal->hSharedMemory);
        free(handleInternal);
        return k_hInvalidIpcHandle;
    }

    std::string mutexName = std::string(args.szSharedMemoryName) + "_FUNC_MTX";
    std::string eventName = std::string(args.szSharedMemoryName) + "_FUNC_EVT";
    handleInternal->hNativeMutex = CreateMutexA(NULL, FALSE, mutexName.c_str());
    handleInternal->hNativeEvent = CreateEventA(NULL, TRUE, FALSE, eventName.c_str());
    if (handleInternal->hNativeMutex == NULL || handleInternal->hNativeEvent == NULL) {
        IpcHandle_t hIpcHandle = ((IpcHandle_t)handleInternal);
        ipc_server_shutdown(&hIpcHandle);
        return k_hInvalidIpcHandle;
    }
#elif IPC_OS_LINUX
    // shared mem needs a leading / on linux
    std::string szSharedMemoryPath = (args.szSharedMemoryName[0] == '/') ? args.szSharedMemoryName : "/" + std::string(args.szSharedMemoryName);

    int hSharedMemFd = shm_open(szSharedMemoryPath.c_str(), O_CREAT | O_RDWR, IPC_S_RW_ALL);
    if (hSharedMemFd == -1) {
        free(handleInternal);
        return k_hInvalidIpcHandle;
    }

    handleInternal->hSharedMemory = (IpcNativeHandle_t)(intptr_t)hSharedMemFd;
    ftruncate(hSharedMemFd, totalSharedMemSize);

    handleInternal->pSharedMemory = mmap(NULL, totalSharedMemSize, PROT_READ | PROT_WRITE, MAP_SHARED, hSharedMemFd, 0);
    if (handleInternal->pSharedMemory == MAP_FAILED) {
        close(hSharedMemFd);
        free(handleInternal);
        return k_hInvalidIpcHandle;
    }

    std::string mutexName = szSharedMemoryPath + "_FUNC_MTX";
    std::string eventName = szSharedMemoryPath + "_FUNC_EVT";

    handleInternal->hNativeMutex = (IpcNativeHandle_t)sem_open(mutexName.c_str(), O_CREAT, IPC_S_RW_ALL, 1);
    handleInternal->hNativeEvent = (IpcNativeHandle_t)sem_open(eventName.c_str(), O_CREAT, IPC_S_RW_ALL, 0);
    if (handleInternal->hNativeMutex == SEM_FAILED || handleInternal->hNativeEvent == SEM_FAILED) {
        IpcHandle_t hIpcHandle = ((IpcHandle_t)handleInternal);
        ipc_server_shutdown(&hIpcHandle);
        return k_hInvalidIpcHandle;
    }
#else
#error "Unsupported platform"
#endif

    for (size_t i = 0; i < args.dwFunctionCount; ++i) {
        const IpcFunction_t& reg = args.aFunctions[i];
        __internal__IpcFunction_t func = {
            .unCmdType = reg.commandType,
            .pfnCallback = reg.callback
        };
        handleInternal->registed_functions.push_back(func);
    }

    // init shared memory to zeros
    memset(handleInternal->pSharedMemory, 0, totalSharedMemSize);

    handleInternal->is_running = true;
    handleInternal->poll_thread = std::thread(__internal_ipc_server_poll_requests, (IpcHandle_t)handleInternal);

    return (IpcHandle_t)handleInternal;
}

IpcHandle_t ipc_client_init(IpcCreateArguments_t args) {
    if (!args.szSharedMemoryName) {
        return k_hInvalidIpcHandle;
    }

    __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)calloc(1, sizeof(__internal__IpcHandle_t));

    if (!handleInternal) {
        return k_hInvalidIpcHandle;
    }

    handleInternal->szSharedMemoryName = args.szSharedMemoryName;

#if IPC_OS_WIN32
    handleInternal->hSharedMemory = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, args.szSharedMemoryName);
    if (handleInternal->hSharedMemory == NULL) {
        free(handleInternal);
        return k_hInvalidIpcHandle;
    }

    handleInternal->pSharedMemory = MapViewOfFile(handleInternal->hSharedMemory, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (handleInternal->pSharedMemory == NULL) {
        CloseHandle(handleInternal->hSharedMemory);
        free(handleInternal);
        return k_hInvalidIpcHandle;
    }

    std::string mutexName = std::string(args.szSharedMemoryName) + "_FUNC_MTX";
    std::string eventName = std::string(args.szSharedMemoryName) + "_FUNC_EVT";
    handleInternal->hNativeMutex = OpenMutexA(SYNCHRONIZE | MUTEX_MODIFY_STATE, FALSE, mutexName.c_str());
    handleInternal->hNativeEvent = OpenEventA(EVENT_ALL_ACCESS, FALSE, eventName.c_str());
    if (handleInternal->hNativeMutex == NULL || handleInternal->hNativeEvent == NULL) {
        IpcHandle_t handleFinal = ((IpcHandle_t)handleInternal);
        ipc_client_shutdown(&handleFinal);
        return k_hInvalidIpcHandle;
    }
#elif IPC_OS_LINUX
    std::string szSharedMemoryPath = (args.szSharedMemoryName[0] == '/') ? args.szSharedMemoryName : "/" + std::string(args.szSharedMemoryName);

    int hSharedMemFd = shm_open(szSharedMemoryPath.c_str(), O_RDWR, IPC_S_RW_ALL);
    if (hSharedMemFd == -1) {
        free(handleInternal);
        return k_hInvalidIpcHandle;
    }

    handleInternal->hSharedMemory = (IpcNativeHandle_t)(intptr_t)hSharedMemFd;
    struct stat st = {};
    fstat(hSharedMemFd, &st);
    handleInternal->pSharedMemory = mmap(NULL, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, hSharedMemFd, 0);

    std::string mutexName = szSharedMemoryPath + "_FUNC_MTX";
    std::string eventName = szSharedMemoryPath + "_FUNC_EVT";
    handleInternal->hNativeMutex = (IpcNativeHandle_t)sem_open(mutexName.c_str(), 1);
    handleInternal->hNativeEvent = (IpcNativeHandle_t)sem_open(eventName.c_str(), 0);
    if (handleInternal->hNativeMutex == SEM_FAILED || handleInternal->hNativeEvent == SEM_FAILED) {
        IpcHandle_t hIpcHandle = ((IpcHandle_t)handleInternal);
        ipc_client_shutdown(&hIpcHandle);
        return k_hInvalidIpcHandle;
    }
#else
#error "Unsupported platform"
#endif

    for (size_t i = 0; i < args.dwFunctionCount; ++i) {
        const IpcFunction_t& reg = args.aFunctions[i];
        __internal__IpcFunction_t func = {
            .unCmdType = reg.commandType,
        };
        handleInternal->registed_functions.push_back(func);
    }

    return (IpcHandle_t)handleInternal;
}

bool ipc_server_shutdown(IpcHandle_t* hIpcServer) {
    if (hIpcServer && *hIpcServer != k_hInvalidIpcHandle) {
        __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)*hIpcServer;

        handleInternal->is_running = false;
        if (handleInternal->poll_thread.joinable()) {
            handleInternal->poll_thread.join();
        }
#if IPC_OS_WIN32

        for (size_t i = 0; i < handleInternal->registed_operations.size(); ++i) {
            if (handleInternal->registed_operations[i].hNativeEvent)
                CloseHandle((HANDLE)handleInternal->registed_operations[i].hNativeEvent);
            if (handleInternal->registed_operations[i].hNativeMutex)
                CloseHandle((HANDLE)handleInternal->registed_operations[i].hNativeMutex);
        }

        if (handleInternal->hNativeMutex)
            CloseHandle((HANDLE)handleInternal->hNativeMutex);
        if (handleInternal->hNativeEvent)
            CloseHandle((HANDLE)handleInternal->hNativeEvent);

        if (handleInternal->pSharedMemory)
            UnmapViewOfFile(handleInternal->pSharedMemory);
        if (handleInternal->hSharedMemory)
            CloseHandle(handleInternal->hSharedMemory);
#elif IPC_OS_LINUX
        std::string szSharedMemoryPath = (handleInternal->szSharedMemoryName[0] == '/') ? handleInternal->szSharedMemoryName : "/" + std::string(handleInternal->szSharedMemoryName);

        if ((sem_t*)handleInternal->hNativeMutex != SEM_FAILED) {
            sem_close((sem_t*)handleInternal->hNativeMutex);
            sem_unlink((szSharedMemoryPath + "_FUNC_MTX").c_str());
        }
        if ((sem_t*)handleInternal->hNativeEvent != SEM_FAILED) {
            sem_close((sem_t*)handleInternal->hNativeEvent);
            sem_unlink((szSharedMemoryPath + "_FUNC_EVT").c_str());
        }

        for (size_t i = 0; i < handleInternal->registed_operations.size(); ++i) {
            IpcOperation_t& op = handleInternal->registed_operations[i];
            std::string opMutexName = szSharedMemoryPath + "_" + std::string(op.szIdentifier) + "_MTX";
            std::string opEventName = szSharedMemoryPath + "_" + std::string(op.szIdentifier) + "_EVT";
            if ((sem_t*)op.hNativeMutex != SEM_FAILED) {
                sem_close((sem_t*)op.hNativeMutex);
                sem_unlink(opMutexName.c_str());
            }
            if ((sem_t*)op.hNativeEvent != SEM_FAILED) {
                sem_close((sem_t*)op.hNativeEvent);
                sem_unlink(opEventName.c_str());
            }
        }

        if (handleInternal->pSharedMemory) {
            struct stat st = {};
            fstat((int)(intptr_t)handleInternal->hSharedMemory, &st);
            munmap(handleInternal->pSharedMemory, st.st_size);
        }
        if (handleInternal->hSharedMemory)
            close((int)(intptr_t)handleInternal->hSharedMemory);
        shm_unlink(szSharedMemoryPath.c_str());
#else
#error "Unsupported platform"
#endif

        free(*hIpcServer);
        *hIpcServer = k_hInvalidIpcHandle;
        return true;
    }
    return false;
}
bool ipc_client_shutdown(IpcHandle_t* hIpcClient) {
    if (hIpcClient && *hIpcClient != k_hInvalidIpcHandle) {
        __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)*hIpcClient;

#if IPC_OS_WIN32
        for (size_t i = 0; i < handleInternal->registed_operations.size(); ++i) {
            if (handleInternal->registed_operations[i].hNativeEvent)
                CloseHandle((HANDLE)handleInternal->registed_operations[i].hNativeEvent);
            if (handleInternal->registed_operations[i].hNativeMutex)
                CloseHandle((HANDLE)handleInternal->registed_operations[i].hNativeMutex);
        }

        if (handleInternal->hNativeMutex)
            CloseHandle((HANDLE)handleInternal->hNativeMutex);
        if (handleInternal->hNativeEvent)
            CloseHandle((HANDLE)handleInternal->hNativeEvent);

        if (handleInternal->pSharedMemory)
            UnmapViewOfFile(handleInternal->pSharedMemory);
        if (handleInternal->hSharedMemory)
            CloseHandle(handleInternal->hSharedMemory);
#elif IPC_OS_LINUX
        std::string szSharedMemoryPath = (handleInternal->szSharedMemoryName[0] == '/') ? handleInternal->szSharedMemoryName : "/" + std::string(handleInternal->szSharedMemoryName);

        for (size_t i = 0; i < handleInternal->registed_operations.size(); ++i) {
            IpcOperation_t& op = handleInternal->registed_operations[i];
            if ((sem_t*)op.hNativeMutex != SEM_FAILED)
                sem_close((sem_t*)op.hNativeMutex);
            if ((sem_t*)op.hNativeEvent != SEM_FAILED)
                sem_close((sem_t*)op.hNativeEvent);
        }

        if ((sem_t*)handleInternal->hNativeMutex != SEM_FAILED)
            sem_close((sem_t*)handleInternal->hNativeMutex);
        if ((sem_t*)handleInternal->hNativeEvent != SEM_FAILED)
            sem_close((sem_t*)handleInternal->hNativeEvent);

        if (handleInternal->pSharedMemory) {
            struct stat st = {};
            fstat((int)(intptr_t)handleInternal->hSharedMemory, &st);
            munmap(handleInternal->pSharedMemory, st.st_size);
        }
        if (handleInternal->hSharedMemory)
            close((int)(intptr_t)handleInternal->hSharedMemory);
#else
#error "Unsupported platform"
#endif

        free(*hIpcClient);
        *hIpcClient = k_hInvalidIpcHandle;
        return true;
    }
    return false;
}

bool ipc_client_dispatch_function(IpcHandle_t hIpcClient, IpcCommandType_t unCmdType, void* pArgs, size_t dwArgsSize) {
    if (hIpcClient == k_hInvalidIpcHandle || dwArgsSize > IPC_FUNCTION_ARGS_TOTAL_SIZE - sizeof(IpcCommandType_t)) {
        return false;
    }
    __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)hIpcClient;

    // unCmdType must be registered
    bool bCmdIsValid = false;
    for (size_t i = 0; i < handleInternal->registed_functions.size(); i++) {
        if (handleInternal->registed_functions[i].unCmdType == unCmdType) {
            bCmdIsValid = true;
            break;
        }
    }
    if (!bCmdIsValid) {
        return false;
    }

#if IPC_OS_WIN32
    WaitForSingleObject((HANDLE)handleInternal->hNativeMutex, INFINITE);
    void* pSharedMem = handleInternal->pSharedMemory;
    *((IpcCommandType_t*)pSharedMem) = unCmdType;
    memcpy((char*)pSharedMem + sizeof(IpcCommandType_t), pArgs, dwArgsSize);
    SetEvent((HANDLE)handleInternal->hNativeEvent);
    ReleaseMutex((HANDLE)handleInternal->hNativeMutex);
#elif IPC_OS_LINUX
    sem_t* semMtx = (sem_t*)handleInternal->hNativeMutex;
    sem_t* semEvt = (sem_t*)handleInternal->hNativeEvent;

    sem_wait(semMtx);
    void* pSharedMem = handleInternal->pSharedMemory;
    *((IpcCommandType_t*)pSharedMem) = unCmdType;
    memcpy((char*)pSharedMem + sizeof(IpcCommandType_t), pArgs, dwArgsSize);
    sem_post(semEvt);
    sem_post(semMtx);
#else
#error "Unsupported platform"
#endif

    return true;
}

bool ipc_server_register_operation(IpcHandle_t hIpcServer, IpcOperation_t* ipcOperation) {
    if (hIpcServer == k_hInvalidIpcHandle || !ipcOperation->szIdentifier) {
        return false;
    }
    __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)hIpcServer;

#if IPC_OS_WIN32
    std::string mutexName = std::string(handleInternal->szSharedMemoryName) + "_" + std::string(ipcOperation->szIdentifier) + "_MTX";
    std::string eventName = std::string(handleInternal->szSharedMemoryName) + "_" + std::string(ipcOperation->szIdentifier) + "_EVT";

    ipcOperation->hNativeMutex = CreateMutexA(NULL, FALSE, mutexName.c_str());
    ipcOperation->hNativeEvent = CreateEventA(NULL, FALSE, FALSE, eventName.c_str()); // Auto-reset event

    if (ipcOperation->hNativeMutex == NULL || ipcOperation->hNativeEvent == NULL) {
        if (ipcOperation->hNativeMutex)
            CloseHandle((HANDLE)ipcOperation->hNativeMutex);
        if (ipcOperation->hNativeEvent)
            CloseHandle((HANDLE)ipcOperation->hNativeEvent);
        return false;
    }
#elif IPC_OS_LINUX
    std::string szSharedMemoryPath = (handleInternal->szSharedMemoryName[0] == '/') ? handleInternal->szSharedMemoryName : "/" + std::string(handleInternal->szSharedMemoryName);
    std::string mutexName = szSharedMemoryPath + "_" + std::string(ipcOperation->szIdentifier) + "_MTX";
    std::string eventName = szSharedMemoryPath + "_" + std::string(ipcOperation->szIdentifier) + "_EVT";

    ipcOperation->hNativeMutex = (IpcNativeHandle_t)sem_open(mutexName.c_str(), O_CREAT, IPC_S_RW_ALL, 1);
    ipcOperation->hNativeEvent = (IpcNativeHandle_t)sem_open(eventName.c_str(), O_CREAT, IPC_S_RW_ALL, 0);

    if (ipcOperation->hNativeMutex == SEM_FAILED || ipcOperation->hNativeEvent == SEM_FAILED) {
        if ((sem_t*)ipcOperation->hNativeMutex != SEM_FAILED) {
            sem_close((sem_t*)ipcOperation->hNativeMutex);
            sem_unlink(mutexName.c_str());
        }
        if ((sem_t*)ipcOperation->hNativeEvent != SEM_FAILED) {
            sem_close((sem_t*)ipcOperation->hNativeEvent);
            sem_unlink(eventName.c_str());
        }
        return false;
    }
#else
#error "Unsupported platform"
#endif

    handleInternal->registed_operations.push_back(*ipcOperation);

    return true;
}

bool ipc_server_write_shared_memory(IpcHandle_t hIpcServer, IpcOperation_t ipcOperation, void* pSrc, size_t destSize) {
    if (hIpcServer == k_hInvalidIpcHandle || !pSrc || destSize == 0) {
        return false;
    }
    if (!ipcOperation.hNativeMutex || !ipcOperation.hNativeEvent)
        return false;
    __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)hIpcServer;

#if IPC_OS_WIN32
    DWORD ret = WaitForSingleObject((HANDLE)ipcOperation.hNativeMutex, 10);
    if (ret == WAIT_TIMEOUT) {
        return false;
    }
    void* pDest = (char*)handleInternal->pSharedMemory + IPC_FUNCTION_ARGS_TOTAL_SIZE + ipcOperation.dwSharedMemoryOffset;
    memcpy(pDest, pSrc, destSize);
    SetEvent((HANDLE)ipcOperation.hNativeEvent);
    ReleaseMutex((HANDLE)ipcOperation.hNativeMutex);
#elif IPC_OS_LINUX
    sem_t* semMtx = (sem_t*)ipcOperation.hNativeMutex;
    sem_t* semEvt = (sem_t*)ipcOperation.hNativeEvent;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += 10000000; // 10ms
    if (ts.tv_nsec >= 1000000000) { ts.tv_sec += 1; ts.tv_nsec -= 1000000000; }

    if (sem_timedwait(semMtx, &ts) != 0) {
        return false;
    }
    void* pDest = (char*)handleInternal->pSharedMemory + IPC_FUNCTION_ARGS_TOTAL_SIZE + ipcOperation.dwSharedMemoryOffset;
    memcpy(pDest, pSrc, destSize);
    sem_post(semEvt);
    sem_post(semMtx);
#else
#error "Unsupported platform"
#endif

    return true;
}

bool ipc_client_read_shared_memory(IpcHandle_t hIpcClient, IpcOperation_t ipcOperation, void* pDest, size_t destSize) {
    if (hIpcClient == k_hInvalidIpcHandle || !pDest || destSize == 0) {
        return false;
    }
    if (!ipcOperation.hNativeMutex || !ipcOperation.hNativeEvent)
        return false;
    __internal__IpcHandle_t* handleInternal = (__internal__IpcHandle_t*)hIpcClient;

#if IPC_OS_WIN32
    DWORD ret = WaitForSingleObject((HANDLE)ipcOperation.hNativeMutex, 10);
    if (ret == WAIT_TIMEOUT) {
        return false;
    }
    void* pSrc = (char*)handleInternal->pSharedMemory + IPC_FUNCTION_ARGS_TOTAL_SIZE + ipcOperation.dwSharedMemoryOffset;
    memcpy(pDest, pSrc, destSize);
    ReleaseMutex((HANDLE)ipcOperation.hNativeMutex);
#elif IPC_OS_LINUX
    sem_t* semMtx = (sem_t*)ipcOperation.hNativeMutex;
    sem_t* semEvt = (sem_t*)ipcOperation.hNativeEvent;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += 10000000;
    if (ts.tv_nsec >= 1000000000) { ts.tv_sec += 1; ts.tv_nsec -= 1000000000; }

    if (sem_timedwait(semMtx, &ts) != 0) {
        return false;
    }
    void* pSrc = (char*)handleInternal->pSharedMemory + IPC_FUNCTION_ARGS_TOTAL_SIZE + ipcOperation.dwSharedMemoryOffset;
    memcpy(pDest, pSrc, destSize);
    sem_post(semMtx);
#else
#error "Unsupported platform"
#endif

    return true;
}

#if IPC_OS_LINUX
#undef IPC_S_RW_ALL
#endif

#endif // IPC_IMPLEMENTATION
#endif // HEKKY_IPC_INC