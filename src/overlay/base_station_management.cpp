#include "base_station_management.h"
#include "log.h"
#include "platform.h"
#include "util.h"
#include <atomic>
#include <condition_variable>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <mutex>
#include <queue>
#include <simplecble/simplecble.h>
#include <string.h>

// use to dump hex data to log
// @NOTE: makes log files significantly larger!
// #define BLUETOOTH_LOGGING_VERBOSE

#if OS_WINDOWS
#include <objbase.h> // need COM for worker thread

// increase BLE poll rate for speed on windows
// https://stackoverflow.com/a/37328965
#include <BluetoothAPIs.h>
#include <Bthsdpdef.h>
#include <windows.h>

const DWORD IOCTL_BLE_SCAN_FREQUENCY = 0x41118C;

struct LE_SCAN_REQUEST_ORIG {
    uint32_t scan_type; // 0 = passive, 1 = active
    uint16_t scan_window;
    uint16_t scan_interval;
};

struct LE_SCAN_REQUEST_WIN10 {
    uint32_t unknown1;
    uint32_t scan_type; // 0 = passive, 1 = active
    uint32_t unknown2;
    uint16_t scan_window;
    uint16_t scan_interval;
    uint32_t unknown3[2];
};

// https://www.bluetooth.com/bluetooth-le-primer/#mcetoc_1iiprfme5g
enum LE_SCAN_REQUEST_PHY_MODE {
    LE_SCAN_PHY_1M = 1, // LE 1M PHY (1 Mbps)
    LE_SCAN_PHY_2M = 2, // LE 2M PHY (2 Mbps)
    LE_SCAN_PHY_CODED = 4 // LE Coded PHY (Long Range)
};

struct LE_SCAN_REQUEST_WIN11 {
    uint32_t override_phy; // 0 = use default, 1 = override, other values fail
    uint32_t scan_type; // 0 = passive, 1 = active
    uint32_t mode_or_phy; // LE_SCAN_REQUEST_PHY_MODE, if and only if use_phy == 1
    uint16_t scan_window; // LE scan window (units of 0.625ms); strictly <= scan_interval
    uint16_t scan_interval; // LE scan interval (units of 0.625ms)
};

#endif // OS_WINDOWS

namespace spacecal {
namespace bluetooth {

    // how many times a job should be attempted for
    constexpr size_t k_MAX_OPERATION_ATTEMPTS = 8;
    constexpr size_t k_RETRY_DELAY_MS = 1000;

    enum EBleJobType_t : uint8_t {
        BleJob_SetChannel,
        BleJob_SetPower,
    };

    struct BleJob_t {
        EBleJobType_t type;
        size_t index;
        uint8_t data; // channel id / power state enum
    };

    // ---- BLE worker thread: owns the WinRT/COM apartment, executes jobs serially ----
    struct BleWorkerState_t {
        std::thread thread;
        std::mutex mutex;
        std::condition_variable cv;
        std::queue<BleJob_t> queue;
        std::atomic<bool> running = false;
    };

    BleWorkerState_t g_ble_worker;

    struct BaseStationInternal_t {
        BaseStation_t base_station;

        simpleble_peripheral_t hBtDevice = nullptr;
        bool isOldFirmware = false;

        uint8_t targetChannel = 0xFF;
        bool isChannelDirty = false;

        EPowerState_t targetPowerState = PowerState_Unknown;
        bool isPowerDirty = false;
    };

    struct BaseStationManagementState_t {
        std::vector<simpleble_adapter_t> aAdapters;
        std::vector<BaseStationInternal_t> aTrackedBaseStations;
        std::mutex mutexBaseStationList;
        size_t _itersElapsed = 0;
        bool bStationsChannelCollision = false;
        std::thread scannerThread;
        std::mutex mutexScanThread;
        std::condition_variable cvScanThread;
        std::atomic<bool> bScanThreadRunning = false;
    };

    BaseStationManagementState_t g_base_station_state;

    // Base station 1.0 GATT characteristics
    constexpr simpleble_uuid_t BASE_STATION_1_POWER_SERVICE_UUID = { "0000cb00-0000-1000-8000-00805f9b34fb" };
    constexpr simpleble_uuid_t BASE_STATION_1_POWER_CHARACTERISTIC_UUID = { "0000cb01-0000-1000-8000-00805f9b34fb" };

    // Base station 2.0 GATT characteristics
    constexpr simpleble_uuid_t BASE_STATION_2_SERVICE_UUID = { "00001523-1212-efde-1523-785feabcd124" };
    constexpr simpleble_uuid_t BASE_STATION_2_MODE_CHARACTERISTIC_UUID = { "00001524-1212-efde-1523-785feabcd124" };
    constexpr simpleble_uuid_t BASE_STATION_2_IDENTIFY_CHARACTERISTIC_UUID = { "00008421-1212-efde-1523-785feabcd124" };
    constexpr simpleble_uuid_t BASE_STATION_2_POWER_CHARACTERISTIC_UUID = { "00001525-1212-efde-1523-785feabcd124" };

    constexpr uint8_t k_LIGHTHOUSE_10_PING_MAGIC = 0x12;
    const uint32_t k_LIGHTHOUSE_10_GENERIC_ID = 0xFFFFFFFF;

    // we need a delay before we can send commands until bluetooth fully initialises
    constexpr size_t k_MIN_ITERS_BEFORE_COMMS = 60;
    constexpr size_t k_ESTIMATED_MAX_BASE_STATION_COUNT = 32;

    // Should do something similar for BS 1.0s
    struct BaseStation20_InfoState {
        char header[2];
        uint8_t channel;
        uint8_t unk1;
        uint8_t power_state;
        uint8_t unk3;
        uint8_t is_faulty; // 1 => faulty (red light)
    };

    // to turn on from standby ->
    enum Lighthouse10Command_t : uint8_t {
        LH10Cmd_PowerOn = 0x00,
        LH10Cmd_Standby = 0x01,
        LH10Cmd_Wakeup = 0x02,
    };

    enum Lighthouse10Channel_t : uint8_t {
        LH10Chn_A = 0x00,
        LH10Chn_B = 0x01,
        LH10Chn_C = 0x02,
    };

    // 1.0 power packet
    struct Lighthouse10_PowerPacket_t {
        uint8_t magic = k_LIGHTHOUSE_10_PING_MAGIC;
        uint8_t command;

        uint16_t timeout; // big endian, seconds
        uint32_t id = k_LIGHTHOUSE_10_GENERIC_ID;

        uint8_t _tail[12];
    };

    void async_set_base_station_10_power_state(size_t index, EPowerState_t target_state, int retry_count = 0);
    void async_set_base_station_20_channel_state(size_t index, uint8_t target_channel, int retry_count = 0);
    void async_set_base_station_20_power_state(size_t index, EPowerState_t target_state, int retry_count = 0);

    void _ble_worker_run_job(const BleJob_t& job)
    {
        switch (job.type) {
        case BleJob_SetChannel:
            async_set_base_station_20_channel_state(job.index, job.data);
            break;
        case BleJob_SetPower: {
            EPowerState_t targetState = (EPowerState_t)job.data;
            auto eType = g_base_station_state.aTrackedBaseStations[job.index].base_station.eType;
            if (eType == BaseStationType_10) {
                async_set_base_station_10_power_state(job.index, targetState);
            } else if (eType == BaseStationType_20) {
                async_set_base_station_20_power_state(job.index, targetState);
            }
            break;
        }
        }
    }

    void _ble_worker_thread_main()
    {
        platform::setThreadName("BluetoothWorkerThread");

#if OS_WINDOWS
        // need COM to be intialised or BLE invocations from this thread wont succeed
        HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        bool comInitialized = SUCCEEDED(hr);
#endif

        for (;;) {
            BleJob_t job;
            {
                std::unique_lock<std::mutex> lock(g_ble_worker.mutex);
                g_ble_worker.cv.wait(lock, [] { return !g_ble_worker.queue.empty() || !g_ble_worker.running; });

                if (!g_ble_worker.running && g_ble_worker.queue.empty()) {
                    break;
                }

                job = g_ble_worker.queue.front();
                g_ble_worker.queue.pop();
            }

            _ble_worker_run_job(job);
        }

#if OS_WINDOWS
        if (comInitialized) {
            CoUninitialize();
        }
#endif
    }

    void ble_worker_start()
    {
        g_ble_worker.running = true;
        g_ble_worker.thread = std::thread(_ble_worker_thread_main);
    }

    void ble_worker_stop()
    {
        {
            std::lock_guard<std::mutex> lock(g_ble_worker.mutex);
            g_ble_worker.running = false;
        }
        g_ble_worker.cv.notify_all();
        if (g_ble_worker.thread.joinable()) {
            g_ble_worker.thread.join();
        }
    }

    void ble_worker_enqueue(BleJob_t job)
    {
        {
            std::lock_guard<std::mutex> lock(g_ble_worker.mutex);
            g_ble_worker.queue.push(job);
        }
        g_ble_worker.cv.notify_one();
    }

    bool is_base_station_10(char* identifier)
    {
        if (!identifier) {
            return false;
        }
        size_t len = strlen(identifier);
        if (len >= 13) {
            if (strncmp(identifier, "HTC BS ", 7) != 0) {
                return false;
            }
            for (int i = 7; i < 13; ++i) {
                if (!isxdigit(identifier[i])) {
                    return false;
                }
            }
            // @NOTE: given 2.0s may enumerate as LHB-00000000 ; im going to assume 1.0s MAY do the same
            if (strncmp(identifier, "HTC BS 000000", 13) == 0) {
                return false;
            }
            return true;
        }
        return false;
    }

    bool is_base_station_20(char* identifier)
    {
        if (!identifier) {
            return false;
        }
        size_t len = strlen(identifier);
        if (len >= 12) {
            if (strncmp(identifier, "LHB-", 4) != 0) {
                return false;
            }
            for (int i = 4; i < 12; ++i) {
                if (!isxdigit(identifier[i])) {
                    return false;
                }
            }
            // @NOTE: we INTENTIONALLY check if the base station ID is LHB-00000000 ; a Base Station 2.0 may appear under that name during startup and mislead the user!
            if (strncmp(identifier, "LHB-00000000", 12) == 0) {
                return false;
            }
            return true;
        }
        return false;
    }

    // appends or updates an entry in g_base_station_state.aTrackedBaseStations
    void add_base_station_to_collection(const BaseStationInternal_t& baseStation)
    {

        bool foundStation = false;
        for (size_t i = 0; i < g_base_station_state.aTrackedBaseStations.size() && !foundStation; i++) {
            if (g_base_station_state.aTrackedBaseStations[i].base_station.szSerialNumber == baseStation.base_station.szSerialNumber) {
                // already present, update this and exit
                g_base_station_state.aTrackedBaseStations[i].hBtDevice = baseStation.hBtDevice;
                g_base_station_state.aTrackedBaseStations[i].isOldFirmware = baseStation.isOldFirmware;
                g_base_station_state.aTrackedBaseStations[i].base_station.szSerialNumber = baseStation.base_station.szSerialNumber;
                g_base_station_state.aTrackedBaseStations[i].base_station.szMacAddress = baseStation.base_station.szMacAddress;
                g_base_station_state.aTrackedBaseStations[i].base_station.channel = baseStation.base_station.channel;
                g_base_station_state.aTrackedBaseStations[i].base_station.eType = baseStation.base_station.eType;
                if (baseStation.base_station.eType == EBaseStationType_t::BaseStationType_20 || (baseStation.base_station.powerState != EPowerState_t::PowerState_Unknown) // prevent overwrite for 1.0s
                ) {
                    g_base_station_state.aTrackedBaseStations[i].base_station.powerState = baseStation.base_station.powerState;
                }
                g_base_station_state.aTrackedBaseStations[i].base_station.isConnected = baseStation.base_station.isConnected;
                g_base_station_state.aTrackedBaseStations[i].base_station.isFirmwareCooked = baseStation.base_station.isFirmwareCooked;
                if (baseStation.base_station.firmwareSupportsStandby != EBaseStation20_StandbySupport_t::StandbySupport_Unknown) {
                    g_base_station_state.aTrackedBaseStations[i].base_station.firmwareSupportsStandby = baseStation.base_station.firmwareSupportsStandby;
                }
                foundStation = true;
            }
        }

        if (!foundStation) {
            g_base_station_state.aTrackedBaseStations.push_back(baseStation);

            // sort them by serial number so that they appear in the same order in the UI every time
            std::sort(
                g_base_station_state.aTrackedBaseStations.begin(),
                g_base_station_state.aTrackedBaseStations.end(),
                [](const BaseStationInternal_t& a, const BaseStationInternal_t& b) {
                    return a.base_station.szSerialNumber < b.base_station.szSerialNumber;
                });
        }

        // channels are unknown with 1.0 base stations, so this only matters for 2.0s
        bool bCollisionDetected = false;
        uint16_t activeChannelsMask = 0; // bitwise mask; we have 16 channels on 2.0s so only up to 16 unique bits (slots) to use

        // build a bitwise mask of the occupied channels, if one is set more than once
        for (const auto& tracked : g_base_station_state.aTrackedBaseStations) {
            if (tracked.base_station.eType == BaseStationType_20) {
                auto channel = tracked.base_station.channel;
                if (IS_BASE_STATION_20_CHANNEL_VALID(channel)) {
                    uint16_t channelBit = (1U << (channel - 1));
                    if (activeChannelsMask & channelBit) {
                        bCollisionDetected = true;
                        break;
                    }
                    activeChannelsMask |= channelBit;
                }
            }
        }
        g_base_station_state.bStationsChannelCollision = bCollisionDetected;
    }

    // returns an entry from g_base_station_state.aTrackedBaseStations or nullptr
    BaseStationInternal_t* find_base_station_from_collection(char* szSerialNumber)
    {
        bool foundStation = false;

        for (size_t i = 0; i < g_base_station_state.aTrackedBaseStations.size() && !foundStation; i++) {
            if (strncmp(g_base_station_state.aTrackedBaseStations[i].base_station.szSerialNumber.data(), szSerialNumber, g_base_station_state.aTrackedBaseStations[i].base_station.szSerialNumber.size()) == 0) {
                // already present, update this and exit
                return &g_base_station_state.aTrackedBaseStations[i];
            }
        }

        return nullptr;
    }

    void _ble_callbackStart(simpleble_adapter_t adapter, void* userdata)
    {
        platform::setThreadName("BluetoothPollThread");
    }

    void _ble_callbackStop(simpleble_adapter_t adapter, void* userdata) { }

    void _ble_callbackDevUpdate(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata)
    {
        // @NOTE: this runs on bt thread
        BaseStationManagementState_t* base_station_state = (BaseStationManagementState_t*)userdata;

        // see if present in list
        char* devDisplayName = simpleble_peripheral_identifier(peripheral); // SN: LHB-XXXXXXXX : HTC BS XXXXXX
        char* devAddr = simpleble_peripheral_address(peripheral); // mac address
        size_t manDataCount = simpleble_peripheral_manufacturer_data_count(peripheral);

        // connection state
        bool isConnected = false;
        bool canConnect = false;
        simpleble_err_t err = simpleble_peripheral_is_connected(peripheral, &isConnected);
        err = simpleble_peripheral_is_connectable(peripheral, &canConnect);

        BaseStationInternal_t* pBaseStation = find_base_station_from_collection(devDisplayName);

        if (is_base_station_20(devDisplayName)) {
            // update state
            uint8_t baseStationChannel = k_INVALID_BASE_STATION_CHANNEL;
            EPowerState_t baseStationPowerState = PowerState_Unknown;
            bool isOnOldFirmware = false;
            bool isCooked = false;

            for (size_t i = 0; i < manDataCount; i++) {
                simpleble_manufacturer_data_t manufacturerData = { 0 };
                simpleble_err_t err = simpleble_peripheral_manufacturer_data_get(peripheral, i, &manufacturerData);
                if (err == SIMPLEBLE_SUCCESS) {
                    // The data stored is base station state
                    BaseStation20_InfoState* infoState = (BaseStation20_InfoState*)manufacturerData.data;

                    baseStationChannel = infoState->channel;
                    baseStationPowerState = (EPowerState_t)infoState->power_state;
                    isOnOldFirmware = pBaseStation ? (pBaseStation->isOldFirmware) : (infoState->power_state == PowerState_Awake_TooOldFirmware);
                    isCooked = infoState->is_faulty == 1; // if one the base station is fucked

#if defined(BLUETOOTH_LOGGING_VERBOSE)
                    std::string szVerboseBaseStationState = fmt::format("MN: Got {} bytes for {} ({}) :: {:02X}",
                        manufacturerData.data_length,
                        devDisplayName,
                        manufacturerData.manufacturer_id,
                        fmt::join(manufacturerData.data, manufacturerData.data + manufacturerData.data_length, " "));

                    LOG_BLUETOOTH_INFO("{}", szVerboseBaseStationState);
#endif // BLUETOOTH_LOGGING_VERBOSE

                    // push to collection
                    add_base_station_to_collection({
                        .base_station = {
                            .szSerialNumber = devDisplayName,
                            .szMacAddress = devAddr,

                            .channel = baseStationChannel,
                            .eType = EBaseStationType_t::BaseStationType_20,
                            .powerState = baseStationPowerState,

                            .isConnected = isConnected,
                            .isFirmwareCooked = isCooked,
                        },

                        .hBtDevice = peripheral,
                        .isOldFirmware = isOnOldFirmware,
                    });
                }
            }

            if (base_station_state->_itersElapsed < k_MIN_ITERS_BEFORE_COMMS) {
                base_station_state->_itersElapsed++;
                goto dev_find_cleanup;
            }
        }

        if (is_base_station_10(devDisplayName)) {
            // 1.0s do not leak state so we can't know what they're doing ; keep track of them and nothing else :/
            // push to collection
            add_base_station_to_collection({
                .base_station = {
                    .szSerialNumber = devDisplayName,
                    .szMacAddress = devAddr,

                    .channel = k_INVALID_BASE_STATION_CHANNEL,
                    .eType = EBaseStationType_t::BaseStationType_10,
                    .powerState = PowerState_Unknown,

                    .isConnected = isConnected,
                },
                .hBtDevice = peripheral,
                .isOldFirmware = false,
            });
        }

    dev_find_cleanup:
        simpleble_free(devDisplayName);
        simpleble_free(devAddr);
    }

    void async_set_base_station_10_power_state(size_t index, EPowerState_t target_state, int retry_count)
    {
        g_base_station_state.mutexBaseStationList.lock();

        if (index >= g_base_station_state.aTrackedBaseStations.size()) {
            g_base_station_state.mutexBaseStationList.unlock();
            return;
        }

        simpleble_peripheral_t hBtDevice = g_base_station_state.aTrackedBaseStations[index].hBtDevice;
        std::string szSerialNumber = g_base_station_state.aTrackedBaseStations[index].base_station.szSerialNumber;
        g_base_station_state.mutexBaseStationList.unlock();
        LOG_BLUETOOTH_INFO("Attempting to execute SET_POWER {} on Base Station 1.0 {}...", (uint8_t)target_state, szSerialNumber);

        bool success = false;
        bool bConnectable = false;
        bool bConnected = false;
        if (simpleble_peripheral_is_connectable(hBtDevice, &bConnectable) == SIMPLEBLE_FAILURE) {
            return;
        }
        if (simpleble_peripheral_is_connected(hBtDevice, &bConnected) == SIMPLEBLE_FAILURE) {
            return;
        }
        if (bConnected || simpleble_peripheral_connect(hBtDevice) == SIMPLEBLE_SUCCESS) {
            Lighthouse10Command_t lh10Cmd = LH10Cmd_PowerOn;
            uint16_t dwTimeout = 0;
            switch (target_state) {
            case PowerState_Awake_TooOldFirmware:
            case PowerState_Awake_From_Sleep:
            case PowerState_Awake_From_Standby:
                lh10Cmd = LH10Cmd_PowerOn;
                dwTimeout = 0;
                break;
            case PowerState_Standby:
            case PowerState_Sleep:
                lh10Cmd = LH10Cmd_Standby;
                dwTimeout = 4;
                break;
            }
            Lighthouse10_PowerPacket_t lighthouse_10_packet = {
                .magic = k_LIGHTHOUSE_10_PING_MAGIC,
                .command = lh10Cmd,
                .timeout = make_be16(dwTimeout),
                .id = k_LIGHTHOUSE_10_GENERIC_ID,
            };

            if (simpleble_peripheral_write_command(hBtDevice, BASE_STATION_1_POWER_SERVICE_UUID, BASE_STATION_1_POWER_CHARACTERISTIC_UUID, (const uint8_t*)&lighthouse_10_packet, sizeof(lighthouse_10_packet)) != SIMPLEBLE_SUCCESS) {
                LOG_BLUETOOTH_WARN("Failed to set {} power state, got BLE failure", szSerialNumber);
            } else {
                success = true;
            }

            simpleble_peripheral_disconnect(hBtDevice);
        }

        if (success) {
            g_base_station_state.mutexBaseStationList.lock();
            if (index < g_base_station_state.aTrackedBaseStations.size()) {
                g_base_station_state.aTrackedBaseStations[index].base_station.powerState = target_state;
            }
            g_base_station_state.mutexBaseStationList.unlock();
        }

        // re-attempt
        if (!success && retry_count < k_MAX_OPERATION_ATTEMPTS) {
            std::thread([index, target_state, retry_count]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(k_RETRY_DELAY_MS));
                async_set_base_station_10_power_state(index, target_state, retry_count + 1);
            }).detach();
        } else if (!success) {
            LOG_BLUETOOTH_ERROR("Max retries reached ({} attempts) for base station on op 1.0 SET_POWER {}, giving up.", k_MAX_OPERATION_ATTEMPTS, szSerialNumber);
        }
    }

    void async_set_base_station_20_channel_state(size_t index, uint8_t target_channel, int retry_count)
    {
        g_base_station_state.mutexBaseStationList.lock();

        if (index >= g_base_station_state.aTrackedBaseStations.size()) {
            g_base_station_state.mutexBaseStationList.unlock();
            return;
        }

        simpleble_peripheral_t hBtDevice = g_base_station_state.aTrackedBaseStations[index].hBtDevice;
        bool bIsOldFirmware = g_base_station_state.aTrackedBaseStations[index].isOldFirmware;
        std::string szSerialNumber = g_base_station_state.aTrackedBaseStations[index].base_station.szSerialNumber;
        g_base_station_state.mutexBaseStationList.unlock();
        LOG_BLUETOOTH_INFO("Attempting to execute SET_CHANNEL {} on Base Station 2.0 {}...", (uint8_t)target_channel, szSerialNumber);

        bool success = false;
        bool bConnectable = false;
        bool bConnected = false;
        if (simpleble_peripheral_is_connectable(hBtDevice, &bConnectable) == SIMPLEBLE_FAILURE) {
            return;
        }
        if (simpleble_peripheral_is_connected(hBtDevice, &bConnected) == SIMPLEBLE_FAILURE) {
            return;
        }
        if (bConnected || simpleble_peripheral_connect(hBtDevice) == SIMPLEBLE_SUCCESS) {
            simpleble_err_t err = SIMPLEBLE_SUCCESS;
            size_t serviceCount = simpleble_peripheral_services_count(hBtDevice);
            std::vector<simpleble_service_t> bufServices(serviceCount);
            for (size_t i = 0; i < serviceCount; i++) {
                err = simpleble_peripheral_services_get(hBtDevice, i, &bufServices[i]);
            }

            // query power characteristic props while we're connected to know which firmware version the base station is on
            bool foundPowerCharacteristic = false;
            for (size_t i = 0; i < serviceCount && !foundPowerCharacteristic; i++) {
                if (strncmp(bufServices[i].uuid.value, BASE_STATION_2_SERVICE_UUID.value, SIMPLEBLE_UUID_STR_LEN) == 0) {
                    // found base station 2.0 service
                    for (size_t j = 0; j < bufServices[i].characteristic_count; j++) {
                        if (strncmp(bufServices[i].characteristics[j].uuid.value, BASE_STATION_2_POWER_CHARACTERISTIC_UUID.value, SIMPLEBLE_UUID_STR_LEN) == 0) {
                            // found base station 2.0 power characteristic
#if defined(BLUETOOTH_LOGGING_VERBOSE)
                            LOG_BLUETOOTH_INFO("PWR CHR: read: {}, writeReq {}, writeCmd: {}, notify: {}, indicate: {}",
                                bufServices[i].characteristics[j].can_read,
                                bufServices[i].characteristics[j].can_write_request,
                                bufServices[i].characteristics[j].can_write_command,
                                bufServices[i].characteristics[j].can_notify,
                                bufServices[i].characteristics[j].can_indicate);
#endif // BLUETOOTH_LOGGING_VERBOSE

                            bIsOldFirmware = !bufServices[i].characteristics[j].can_read;

                            foundPowerCharacteristic = true;
                            break;
                        }
                    }
                }
            }

            uint8_t writeData[] = { target_channel };
            size_t writeDataSize = sizeof(writeData) / sizeof(writeData[0]);
            err = simpleble_peripheral_write_command(hBtDevice, BASE_STATION_2_SERVICE_UUID, BASE_STATION_2_MODE_CHARACTERISTIC_UUID, writeData, writeDataSize);
            if (err == SIMPLEBLE_SUCCESS) {
                success = true;
            }

            err = simpleble_peripheral_disconnect(hBtDevice);
            if (err != SIMPLEBLE_SUCCESS) {
                LOG_BLUETOOTH_WARN("Failed to disconnect from base station {}...", szSerialNumber);
            } else {
                LOG_BLUETOOTH_INFO("Set base station {} to channel {}!", szSerialNumber, target_channel);
            }
        }

        if (success) {
            g_base_station_state.mutexBaseStationList.lock();
            if (index < g_base_station_state.aTrackedBaseStations.size()) {
                auto& final_station = g_base_station_state.aTrackedBaseStations[index];
                final_station.base_station.channel = target_channel;
                final_station.base_station.firmwareSupportsStandby = bIsOldFirmware ? EBaseStation20_StandbySupport_t::StandbySupport_Unavailable : EBaseStation20_StandbySupport_t::StandbySupport_Supported;
                final_station.isOldFirmware = bIsOldFirmware;
            }
            g_base_station_state.mutexBaseStationList.unlock();
        }

        // re-attempt
        if (!success && retry_count < k_MAX_OPERATION_ATTEMPTS) {
            std::thread([index, target_channel, retry_count]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(k_RETRY_DELAY_MS));
                async_set_base_station_20_channel_state(index, target_channel, retry_count + 1);
            }).detach();
        } else if (!success) {
            LOG_BLUETOOTH_ERROR("Max retries reached ({} attempts) for base station on op 2.0 SET_CHANNEL {}, giving up.", k_MAX_OPERATION_ATTEMPTS, szSerialNumber);
        }
    }

    void async_set_base_station_20_power_state(size_t index, EPowerState_t target_state, int retry_count)
    {
        g_base_station_state.mutexBaseStationList.lock();

        if (index >= g_base_station_state.aTrackedBaseStations.size()) {
            g_base_station_state.mutexBaseStationList.unlock();
            return;
        }

        simpleble_peripheral_t hBtDevice = g_base_station_state.aTrackedBaseStations[index].hBtDevice;
        bool bIsOldFirmware = g_base_station_state.aTrackedBaseStations[index].isOldFirmware;
        std::string szSerialNumber = g_base_station_state.aTrackedBaseStations[index].base_station.szSerialNumber;
        g_base_station_state.mutexBaseStationList.unlock();
        LOG_BLUETOOTH_INFO("Attempting to execute SET_POWER {} on Base Station 2.0 {}...", (uint8_t)target_state, szSerialNumber);

        bool success = false;
        bool bConnectable = false;
        bool bConnected = false;
        if (simpleble_peripheral_is_connectable(hBtDevice, &bConnectable) == SIMPLEBLE_FAILURE) {
            return;
        }
        if (simpleble_peripheral_is_connected(hBtDevice, &bConnected) == SIMPLEBLE_FAILURE) {
            return;
        }
        if (bConnected || simpleble_peripheral_connect(hBtDevice) == SIMPLEBLE_SUCCESS) {

            simpleble_err_t err = SIMPLEBLE_SUCCESS;

            uint8_t writeData[] = { target_state };
            size_t writeDataSize = sizeof(writeData) / sizeof(writeData[0]);

            simpleble_characteristic_t powerChar;

            size_t serviceCount = simpleble_peripheral_services_count(hBtDevice);
            std::vector<simpleble_service_t> bufServices(serviceCount);
            for (size_t i = 0; i < serviceCount; i++) {
                err = simpleble_peripheral_services_get(hBtDevice, i, &bufServices[i]);
            }

            bool foundPowerCharacteristic = false;
            for (size_t i = 0; i < serviceCount && !foundPowerCharacteristic; i++) {
                if (strncmp(bufServices[i].uuid.value, BASE_STATION_2_SERVICE_UUID.value, SIMPLEBLE_UUID_STR_LEN) == 0) {
                    // found base station 2.0 service
                    for (size_t j = 0; j < bufServices[i].characteristic_count; j++) {
                        if (strncmp(bufServices[i].characteristics[j].uuid.value, BASE_STATION_2_POWER_CHARACTERISTIC_UUID.value, SIMPLEBLE_UUID_STR_LEN) == 0) {
                            // found base station 2.0 power characteristic
#if defined(BLUETOOTH_LOGGING_VERBOSE)
                            LOG_BLUETOOTH_INFO("PWR CHR: read: {}, writeReq {}, writeCmd: {}, notify: {}, indicate: {}",
                                bufServices[i].characteristics[j].can_read,
                                bufServices[i].characteristics[j].can_write_request,
                                bufServices[i].characteristics[j].can_write_command,
                                bufServices[i].characteristics[j].can_notify,
                                bufServices[i].characteristics[j].can_indicate);
#endif // BLUETOOTH_LOGGING_VERBOSE

                            bIsOldFirmware = !bufServices[i].characteristics[j].can_read;

                            powerChar = bufServices[i].characteristics[j];
                            foundPowerCharacteristic = true;
                            break;
                        }
                    }
                }
            }

            bool isValidCommand = true;

            // 2.0s have 2 firmwares which are relavant, old and new firmware
            //
            // new firmware commands and data:
            // Awake:
            //   0x01
            // Standby:
            //   0x02
            // Sleep:
            //   0x01 ; followed by a separate packet with 0x00
            //
            // old firmware commands and data:
            // Awake:
            //   0x09
            // Standby:
            //   0x02
            // Sleep:
            //   0x00 ; followed by a separate packet with 0x00

            // We can query services and depending on the characteristics
            // props of BASE_STATION_2_POWER_CHARACTERISTIC_UUID
            // we can tell if we're on new or old FW
            if (!bIsOldFirmware) {
                switch (target_state) {
                case PowerState_Awake_TooOldFirmware:
                case PowerState_Awake_From_Standby:
                case PowerState_Awake_From_Sleep:
                    writeData[0] = 0x01;
                    break;
                case PowerState_Standby: // unavailable on old firmware
                    writeData[0] = 0x02;
                    break;
                case PowerState_Sleep:
                    writeData[0] = 0x01;
                    break;
                default:
                    isValidCommand = false;
                    LOG_BLUETOOTH_WARN("Got invalid command {:02X} for base station {}, ignoring...", (uint8_t)target_state, szSerialNumber);
                    break;
                }
            } else {
                switch (target_state) {
                case PowerState_Standby: // remap to sleep on old firmware 2.0s
                    writeData[0] = PowerState_Sleep;
                    break;
                }
            }

            if (isValidCommand) {
                err = simpleble_peripheral_write_command(hBtDevice, BASE_STATION_2_SERVICE_UUID, BASE_STATION_2_POWER_CHARACTERISTIC_UUID, writeData, writeDataSize);
                if (err == SIMPLEBLE_SUCCESS) {
                    success = true;
                }

                if (target_state == PowerState_Sleep) {
                    // Sleep cmd requires to send 0x01 followed by 0x00
                    writeData[0] = 0x00;
                    err = simpleble_peripheral_write_command(hBtDevice, BASE_STATION_2_SERVICE_UUID, BASE_STATION_2_POWER_CHARACTERISTIC_UUID, writeData, writeDataSize);
                    if (err == SIMPLEBLE_SUCCESS) {
                        success = true;
                    } else {
                        success = false;
                    }
                }
            }

            err = simpleble_peripheral_disconnect(hBtDevice);
            if (err != SIMPLEBLE_SUCCESS) {
                LOG_BLUETOOTH_WARN("Failed to disconnect from base station {}...", szSerialNumber);
            } else {
                const char* szPowerState = "Active";
                switch (target_state) {
                case spacecal::bluetooth::PowerState_Sleep:
                    szPowerState = "Sleep";
                    break;
                case spacecal::bluetooth::PowerState_Standby:
                    szPowerState = "Standby";
                    break;
                case spacecal::bluetooth::PowerState_Awake_TooOldFirmware:
                case spacecal::bluetooth::PowerState_Awake_From_Sleep:
                case spacecal::bluetooth::PowerState_Awake_From_Standby:
                    szPowerState = "Awake";
                    break;
                case spacecal::bluetooth::PowerState_Unknown:
                default:
                    szPowerState = "Unknown";
                    break;
                }
                LOG_BLUETOOTH_INFO("Set base station {} to power state {}!", szSerialNumber, szPowerState);
            }
        }

        if (success) {
            g_base_station_state.mutexBaseStationList.lock();
            if (index < g_base_station_state.aTrackedBaseStations.size()) {
                auto& final_station = g_base_station_state.aTrackedBaseStations[index];
                final_station.base_station.powerState = target_state;
                final_station.base_station.firmwareSupportsStandby = bIsOldFirmware ? EBaseStation20_StandbySupport_t::StandbySupport_Unavailable : EBaseStation20_StandbySupport_t::StandbySupport_Supported;
                final_station.isOldFirmware = bIsOldFirmware;
            }
            g_base_station_state.mutexBaseStationList.unlock();
        }

        // re-attempt
        if (!success && retry_count < k_MAX_OPERATION_ATTEMPTS) {
            std::thread([index, target_state, retry_count]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(k_RETRY_DELAY_MS));
                async_set_base_station_20_power_state(index, target_state, retry_count + 1);
            }).detach();
        } else if (!success) {
            LOG_BLUETOOTH_ERROR("Max retries reached ({} attempts) for base station on op 2.0 SET_POWER {}, giving up.", k_MAX_OPERATION_ATTEMPTS, szSerialNumber);
        }
    }

    bool is_bluetooth_available()
    {
        return simpleble_adapter_is_bluetooth_enabled();
    }

#if OS_WINDOWS
    bool _try_scan_device_io_control(HANDLE hBtRadio, OVERLAPPED ov, void* pReq, DWORD reqSize)
    {
        ResetEvent(ov.hEvent);
        DWORD dwBytesWritten = 0;

        BOOL bOk = DeviceIoControl(hBtRadio, IOCTL_BLE_SCAN_FREQUENCY, pReq, reqSize, NULL, 0, &dwBytesWritten, &ov);
        if (bOk) {
            return true;
        }

        DWORD dwErr = GetLastError();
        if (dwErr == ERROR_IO_PENDING) {
            // check if overlapped io is allowing async
            DWORD dwWait = WaitForSingleObject(ov.hEvent, 2000);
            if (dwWait != WAIT_OBJECT_0) {
                CancelIoEx(hBtRadio, &ov);
                return false;
            }
            DWORD dwTransferred = 0;
            return GetOverlappedResult(hBtRadio, &ov, &dwTransferred, FALSE) != 0;
        }

        if (dwErr == ERROR_INVALID_PARAMETER) {
            // fallback to blocking
            return DeviceIoControl(hBtRadio, IOCTL_BLE_SCAN_FREQUENCY, pReq, reqSize, NULL, 0, &dwBytesWritten, NULL) != 0;
        }

        return false;
    }

    void _apply_ble_scan_frequency(HANDLE hBtRadio)
    {
        platform::setThreadName("BluetoothScanFreqThread");

        OVERLAPPED ov = { 0 };
        ov.hEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
        if (!ov.hEvent) {
            LOG_BLUETOOTH_WARN("Failed to create event for BLE scan frequency IOCTL, skipping");
            CloseHandle(hBtRadio);
            return;
        }

        // the LE_SCAN_REQUEST struct varies across windows version so fuck it try all of them lmao
        LE_SCAN_REQUEST_WIN11 scanReqWin11 = { .scan_type = 0, .scan_window = 29, .scan_interval = 29 };
        LE_SCAN_REQUEST_WIN10 scanReqWin10 = { .scan_type = 0, .scan_window = 29, .scan_interval = 29 };
        LE_SCAN_REQUEST_ORIG scanReqLegacy = { .scan_type = 0, .scan_window = 29, .scan_interval = 29 };

        if (!_try_scan_device_io_control(hBtRadio, ov, &scanReqWin11, sizeof(scanReqWin11))) {
            if (!_try_scan_device_io_control(hBtRadio, ov, &scanReqWin10, sizeof(scanReqWin10))) {
                if (!_try_scan_device_io_control(hBtRadio, ov, &scanReqLegacy, sizeof(scanReqLegacy))) {
                    LOG_BLUETOOTH_WARN("Failed to increase BLE scan frequency. Bluetooth management may be slower. Error: {}", GetLastError());
                }
            }
        }

        CloseHandle(ov.hEvent);
        CloseHandle(hBtRadio);
    }
#endif // OS_WINDOWS

    void _scan_init_thread_main()
    {
        platform::setThreadName("BluetoothScanInitThread");

#if OS_WINDOWS
        // need COM to be initialised or BLE invocations from this thread wont succeed
        HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        bool comInitialized = SUCCEEDED(hr);
#endif

        bool bSuccess = true;
        if (g_base_station_state.aAdapters.empty()) {
            g_base_station_state.aAdapters.resize(simpleble_adapter_get_count());
            g_base_station_state.aTrackedBaseStations.reserve(k_ESTIMATED_MAX_BASE_STATION_COUNT);
        }

        // Scan bluetooth adapters
        for (size_t i = 0; i < g_base_station_state.aAdapters.size(); i++) {
            g_base_station_state.aAdapters[i] = simpleble_adapter_get_handle(i);
        }

#if OS_WINDOWS
        // Tell the windows API to increase the scan frequency because I like speed
        BLUETOOTH_FIND_RADIO_PARAMS btFindRadioParam = { .dwSize = sizeof(BLUETOOTH_FIND_RADIO_PARAMS) };
        HANDLE hBtRadio = INVALID_HANDLE_VALUE;
        HBLUETOOTH_RADIO_FIND findResult = BluetoothFindFirstRadio(&btFindRadioParam, &hBtRadio);
        if (findResult != NULL) {
            BluetoothFindRadioClose(findResult);
            // DeviceIoControl is blocking here so throw it in it's own thread to not block the main or bluetooth threads
            std::thread(_apply_ble_scan_frequency, hBtRadio).detach();
        }
#endif
        ble_worker_start();

        // Scan for stuff
        for (size_t i = 0; i < g_base_station_state.aAdapters.size(); i++) {
            simpleble_adapter_t adapter = g_base_station_state.aAdapters[i];
            bSuccess = bSuccess && (simpleble_adapter_set_callback_on_scan_start(adapter, _ble_callbackStart, (void*)&g_base_station_state) == simpleble_err_t::SIMPLEBLE_SUCCESS);
            bSuccess = bSuccess && (simpleble_adapter_set_callback_on_scan_stop(adapter, _ble_callbackStop, (void*)&g_base_station_state) == simpleble_err_t::SIMPLEBLE_SUCCESS);
            bSuccess = bSuccess && (simpleble_adapter_set_callback_on_scan_found(adapter, _ble_callbackDevUpdate, (void*)&g_base_station_state) == simpleble_err_t::SIMPLEBLE_SUCCESS);
            bSuccess = bSuccess && (simpleble_adapter_set_callback_on_scan_updated(adapter, _ble_callbackDevUpdate, (void*)&g_base_station_state) == simpleble_err_t::SIMPLEBLE_SUCCESS);

            // Scan begin scanning
            bSuccess = bSuccess && (simpleble_adapter_scan_start(adapter) == simpleble_err_t::SIMPLEBLE_SUCCESS);
        }

        if (!bSuccess) {
            LOG_BLUETOOTH_WARN("Failed to fully initialise base station BLE scanning");
        }

        g_base_station_state.bScanThreadRunning = true;

        {
            std::unique_lock<std::mutex> lock(g_base_station_state.mutexScanThread);
            g_base_station_state.cvScanThread.wait(lock, [] {
                return !g_base_station_state.bScanThreadRunning;
            });
        }

#if OS_WINDOWS
        if (comInitialized) {
            CoUninitialize();
        }
#endif
    }

    bool init_base_station_management()
    {
        g_base_station_state.scannerThread = std::thread(_scan_init_thread_main);
        return true;
    }

    bool shutdown_base_station_management()
    {
        bool bSuccess = true;

        for (size_t i = 0; i < g_base_station_state.aAdapters.size(); i++) {
            simpleble_adapter_t adapter = g_base_station_state.aAdapters[i];
            bSuccess = bSuccess && (simpleble_adapter_scan_stop(adapter) == simpleble_err_t::SIMPLEBLE_SUCCESS);
        }

        {
            std::lock_guard<std::mutex> lock(g_base_station_state.mutexScanThread);
            g_base_station_state.bScanThreadRunning = false;
        }
        g_base_station_state.cvScanThread.notify_all();

        if (g_base_station_state.scannerThread.joinable()) {
            g_base_station_state.scannerThread.join();
        }

        ble_worker_stop();

        if (g_base_station_state.aTrackedBaseStations.size() > 0) {
            for (size_t i = 0; i < g_base_station_state.aTrackedBaseStations.size(); i++) {
                if (g_base_station_state.aTrackedBaseStations[i].base_station.isConnected) {
                    LOG_BLUETOOTH_INFO("Disconnecting from base station {}", g_base_station_state.aTrackedBaseStations[i].base_station.szSerialNumber);
                    bSuccess = bSuccess && (simpleble_peripheral_disconnect(g_base_station_state.aTrackedBaseStations[i].hBtDevice) == simpleble_err_t::SIMPLEBLE_SUCCESS);
                }
            }
        }

        return bSuccess;
    }

    size_t get_base_station_count()
    {
        std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
        return g_base_station_state.aTrackedBaseStations.size();
    }

    BaseStation_t get_base_station(size_t index)
    {
        BaseStation_t station = {};
        std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
        if (g_base_station_state.aTrackedBaseStations.size() > 0 && index < g_base_station_state.aTrackedBaseStations.size()) {
            return g_base_station_state.aTrackedBaseStations[index].base_station;
        }
        return station;
    }

    bool set_base_station_channel(size_t index, uint8_t channel)
    {
        size_t dwBaseStationCount = 0;
        EBaseStationType_t eType;
        std::string szSerialNumber;
        {
            std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
            dwBaseStationCount = g_base_station_state.aTrackedBaseStations.size();
            if (dwBaseStationCount > 0 && index < dwBaseStationCount) {
                eType = g_base_station_state.aTrackedBaseStations[index].base_station.eType;
                szSerialNumber = g_base_station_state.aTrackedBaseStations[index].base_station.szSerialNumber;
            }
        }
        if (dwBaseStationCount > 0 && index < dwBaseStationCount) {
            if (eType == BaseStationType_10) {
                LOG_BLUETOOTH_WARN("Attempted to assign channel {0} on Base Station 1.0 {1}, but the operation is unsupported as it's a 1.0 Base Station! Ignoring...", channel, szSerialNumber);
            } else if (eType == BaseStationType_20) {
                ble_worker_enqueue({ BleJob_SetChannel, index, channel });
            }
        }
        return true;
    }

    bool set_base_station_power_state(size_t index, EPowerState_t state)
    {
        size_t dwBaseStationCount = 0;
        {
            std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
            dwBaseStationCount = g_base_station_state.aTrackedBaseStations.size();
        }
        if (dwBaseStationCount > 0 && index < dwBaseStationCount) {
            ble_worker_enqueue({ BleJob_SetPower, index, (uint8_t)state });
        }
        return true;
    }

    bool set_all_base_station_power_state(EPowerState_t state)
    {
        bool bResult = true;

        size_t dwBaseStationCount = 0;
        {
            std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
            dwBaseStationCount = g_base_station_state.aTrackedBaseStations.size();
        }
        for (size_t i = 0; i < dwBaseStationCount; i++) {
            bResult = bResult && set_base_station_power_state(i, state);
        }

        return bResult && dwBaseStationCount > 0;
    }

    bool auto_assign_base_station_channels()
    {
        bool bResult = true;

        if (!do_base_station_channels_collide()) {
            return true;
        }

        // we can only assign channels to 2.0s, so ignore all 1.0s
        int base_station_20_count = 0;
        {
            std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
            for (size_t i = 0; i < g_base_station_state.aTrackedBaseStations.size(); i++) {
                if (g_base_station_state.aTrackedBaseStations[i].base_station.eType == BaseStationType_20) {
                    base_station_20_count++;
                }
            }
        }

        // we shouldn't touch the channel if there's only a single 2.0 or no 2.0s
        if (base_station_20_count < 2) {
            return true;
        }

        int v2_index = 0;
        double step = (base_station_20_count > 1) ? (15.0 / (base_station_20_count - 1)) : 0.0;

        // distribute channels across channels 1 through 16
        size_t dwBaseStationCount = 0;
        {
            std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
            dwBaseStationCount = g_base_station_state.aTrackedBaseStations.size();
        }
        for (size_t i = 0; i < g_base_station_state.aTrackedBaseStations.size(); i++) {
            BaseStationInternal_t* station = nullptr;
            {
                std::lock_guard<std::mutex> lock(g_base_station_state.mutexBaseStationList);
                station = &g_base_station_state.aTrackedBaseStations[i];
            }
            if (station->base_station.eType != BaseStationType_20) {
                continue;
            }

            int ideal_channel = 1;
            if (base_station_20_count > 1) {
                ideal_channel = (int)(std::round(1.0 + (v2_index * step)));
            }

            if (ideal_channel < 1)
                ideal_channel = 1;
            if (ideal_channel > 16)
                ideal_channel = 16;

            if (station->base_station.channel != ideal_channel) {
                set_base_station_channel(i, ideal_channel);
            }

            v2_index++;
        }

        return bResult;
    }

    bool do_base_station_channels_collide()
    {
        return g_base_station_state.bStationsChannelCollision;
    }
} // bluetooth
} // spacecal