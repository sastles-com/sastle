#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>

// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// #include "ESPNowCam.h"



#define PSRAM_LOCK_MAX_COUNT 16

class PSRAMLock {
  private:
    const int maxLockCount = PSRAM_LOCK_MAX_COUNT;
    int lockCount;
    void* _lockList[PSRAM_LOCK_MAX_COUNT];

  public:
    PSRAMLock() {
      lockCount = 0;
      for (int i = 0; i < maxLockCount; i++) {
        _lockList[i] = NULL;
      }
    }

    void Lock(void) {
      for (int i = 0; i < maxLockCount; i++) {
        _lockList[i] = heap_caps_malloc(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM), MALLOC_CAP_SPIRAM);
        Serial.printf(" _lockList[i] = %8lX\n", _lockList[i]);
        if (_lockList[i] == NULL) {
          break;
        }
        lockCount++;
      }
    }

    void Unlock(void) {
      for (int i = 0; i < lockCount; i++) {
        free(_lockList[i]);
        _lockList[i] = NULL;
      }
      lockCount = 0;
    }
};

PSRAMLock PsramLock;

void printMem() {
  Serial.printf("heap_caps_get_free_size(MALLOC_CAP_SPIRAM)            : %6d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM) );
  Serial.printf("heap_caps_get_free_size(MALLOC_CAP_INTERNAL)          : %6d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL) );
  Serial.printf("heap_caps_get_free_size(MALLOC_CAP_DEFAULT)           : %6d\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT) );
  Serial.printf("heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)   : %6d\n", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) );
  Serial.printf("heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) : %6d\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) );
  Serial.printf("heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)  : %6d\n", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT) );
  Serial.println();
}

void setup(void){
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    auto cfg = M5.config();
    M5.begin(cfg);

    Serial.begin(115200);
    delay(500);

    const char* name;
    switch (M5.getBoard()) {
        case m5::board_t::board_M5StackCoreS3:  name = "StackS3";     break;
        case m5::board_t::board_M5AtomS3Lite:   name = "ATOMS3Lite";  break;
        case m5::board_t::board_M5AtomS3:       name = "ATOMS3";      break;
        case m5::board_t::board_M5StampC3:      name = "StampC3";     break;
        case m5::board_t::board_M5StampS3:      name = "StampS3";     break;
        case m5::board_t::board_M5StampC3U:     name = "StampC3U";    break;
        case m5::board_t::board_M5Stack:        name = "Stack";       break;
        case m5::board_t::board_M5StackCore2:   name = "StackCore2";  break;
        case m5::board_t::board_M5StickC:       name = "StickC";      break;
        case m5::board_t::board_M5StickCPlus:   name = "StickCPlus";  break;
        case m5::board_t::board_M5StackCoreInk: name = "CoreInk";     break;
        case m5::board_t::board_M5Paper:        name = "Paper";       break;
        case m5::board_t::board_M5Tough:        name = "Tough";       break;
        case m5::board_t::board_M5Station:      name = "Station";     break;
        case m5::board_t::board_M5Atom:         name = "ATOM";        break;
        case m5::board_t::board_M5AtomPsram:    name = "ATOM PSRAM";  break;
        case m5::board_t::board_M5AtomU:        name = "ATOM U";      break;
        case m5::board_t::board_M5TimerCam:     name = "TimerCamera"; break;
        case m5::board_t::board_M5StampPico:    name = "StampPico";   break;
        case m5::board_t::board_M5AtomS3R:      name = "M5somS3R";    break;
        case m5::board_t::board_M5AtomS3U:      name = "M5AtomS3U";    break;

        default:                                name = "Who am I ?";  break;
    }
    M5.Display.setTextSize(2); 
    M5.Display.print(name); 
    Serial.println(name);    

    // PSRAMの初期化
    if (!psramInit()) {
        Serial.println("PSRAM初期化失敗");
    } else {
        Serial.println("PSRAM初期化成功");
        Serial.printf("Total PSRAM : %u\n", ESP.getPsramSize());
        Serial.printf("Free  PSRAM : %u\n", ESP.getFreePsram());
    }

    void* psram_buffer = heap_caps_malloc(1024 * 1024, MALLOC_CAP_SPIRAM);
    if (psram_buffer == NULL) {
        Serial.println("PSRAMからメモリ確保失敗");
    } else {
        Serial.println("PSRAMからメモリ確保成功");
        Serial.printf("Total PSRAM : %u\n", ESP.getPsramSize());
        Serial.printf("Free  PSRAM : %u\n", ESP.getFreePsram());
    }    

}
