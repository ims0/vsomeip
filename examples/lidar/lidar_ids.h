#include <stdint.h>
const uint8_t   kCurrentMajor = 0x01;
const uint32_t  kCurrentMinor = 0x00000000;

enum class lidar_type{
    unuse = 0,
    robosense,
    huawei,
    size
};

/* common inst id */
const uint16_t kLeftInstanceId        = 0x0001;  //common
const uint16_t kRightInstanceId       = 0x0002;  //common

enum class lidar_id{
    left = kLeftInstanceId,
    right,
    size
};
struct MulticastAddr{
  const char* multi_cast_address;
  uint16_t msop_port;
};
const char* const kLeftLidarIP    = "172.20.1.52";
const char* const kRightLidarIP    = "172.20.1.53";
/* G3 */
const char* const kLeftPointMultiAddr    = "239.172.20.2";
const uint16_t kLeftPointMultiPort = 55553;
/* G4 */
const char* const kRightPointMultiAddr    = "239.172.20.3";
const uint16_t kRightPointMultiPort = 55554;

/* lidar event*/
const uint16_t kEventGroupId          = 0x0001;
const uint16_t kEventId               = 0x9001;  //method

/* robosense lidar service*/
const uint16_t kPointCloudServiceId   = 0x4030;  // L:G3, R:G4
const uint16_t kLidarStatusServiceId  = 0x4032;  // G5
const uint16_t kRoboServiceRequestId  = 0x4034;
const uint16_t kVehicleStatusInfoData = 0x4036;  //

/* robosense lidar method*/
const uint16_t kSetLidarWorkMode      = 0x0001;
const uint16_t kLidarHeatRequest      = 0x0002;
const uint16_t kSetLidarWorkFrameRate = 0x0003;
const uint16_t kSetLidarROIType       = 0x0004;


