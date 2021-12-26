#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"

#include "vl53l1_api.h"
#include "vl53l1_platform.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

static VL53L1_Dev_t laser_dev;

#define CHECK_STATUS(FAIL_MSG) if (Status != VL53L1_ERROR_NONE) { \
  puts("FAIL: " FAIL_MSG); \
  sleep_ms(5000); \
  reset_usb_boot(0, 0); }

void laser_init()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;

  laser_dev.I2cDevAddr = 0x29;
  Status = VL53L1_software_reset(&laser_dev);
  CHECK_STATUS("Software Reset");

  laser_dev.I2cDevAddr = 0x29;
  Status = VL53L1_WaitDeviceBooted(&laser_dev);
  CHECK_STATUS("Device Boot");

  Status = VL53L1_DataInit(&laser_dev);
  CHECK_STATUS("Data Init");

  Status = VL53L1_StaticInit(&laser_dev);
  CHECK_STATUS("Static Init");

  VL53L1_DeviceInfo_t DeviceInfo;
  Status = VL53L1_GetDeviceInfo(&laser_dev, &DeviceInfo);
  CHECK_STATUS("Device Info");
            printf("VL53L0X_GetDeviceInfo:\n");
            printf("Device Name : %s\n", DeviceInfo.Name);
            printf("Device Type : %s\n", DeviceInfo.Type);
            printf("Device ID : %s\n", DeviceInfo.ProductId);
            printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

  VL53L1_PerformRefSpadManagement(&laser_dev);
  CHECK_STATUS("PerformRefSpadManagement");

  VL53L1_SetXTalkCompensationEnable(&laser_dev, 0);
  CHECK_STATUS("SetXTalkCompensationEnable");
}

void laser_start_ranging(int mode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  if (mode > 0) {
    Status = VL53L1_SetDistanceMode(&laser_dev, mode);
    CHECK_STATUS("Set Distance Mode");
  }
  Status = VL53L1_StartMeasurement(&laser_dev);
  CHECK_STATUS("Start Ranging");
}

static VL53L1_RangingMeasurementData_t RangingMeasurementData;

float laser_get_distance()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  
  Status = VL53L1_WaitMeasurementDataReady(&laser_dev);
  CHECK_STATUS("Wait For Data");
  Status = VL53L1_GetRangingMeasurementData(&laser_dev, &RangingMeasurementData);
  CHECK_STATUS("Get Data");
  float dist = RangingMeasurementData.RangeMilliMeter * 0.001f;
  VL53L1_ClearInterruptAndStartMeasurement(&laser_dev);
  return dist;
}

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 100*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

 sleep_ms(3000);

 laser_init();
 laser_start_ranging(2);

 while (1) {
   float dist = laser_get_distance();
   printf("Dist: %.2f\n", dist);
   sleep_ms(300);
 }

    return 0;
}
