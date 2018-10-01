#ifndef UBX_h
#define UBX_h

#include "Arduino.h"				
//#define dbug Serial

#define HEADER_LENGTH 2
#define PREAMBULE_LENGTH 4
#define CHKSUM_LENGTH 2
#define MAX_FRAME_LENGTH 300
class UBLOX{
  public:
    UBLOX(HardwareSerial& bus,uint32_t baud);
    void begin();
    bool readSensor();

    enum FixType {
      NO_FIX = 0,
      DEAD_RECKONING,
      FIX_2D,
      FIX_3D,
      GNSS_AND_DEAD_RECKONING,
      TIME_ONLY
    };
    enum PowerSaveMode {
      NOT_ACTIVE = 0,
      ENABLED,
      ACQUISITION,
      TRACKING,
      OPTIMIZED_TRACKING,
      INACTIVE
    };
    enum CarrierPhaseStatus {
      NO_SOL = 0,
      FLOAT_SOL,
      FIXED_SOL
    };
    
    uint32_t getTow_ms();
    uint16_t getYear();
    uint8_t getMonth();
    uint8_t getDay();
    uint8_t getHour();
    uint8_t getMin();
    uint8_t getSec();
    int32_t getNanoSec();
    uint8_t getNumSatellites();
    double getLongitude_deg();
    double getLatitude_deg();
    double getEllipsoidHeight_ft();
    double getMSLHeight_ft();
    double getHorizontalAccuracy_ft();
    double getVerticalAccuracy_ft();
    double getNorthVelocity_fps();
    double getEastVelocity_fps();
    double getDownVelocity_fps();
    double getGroundSpeed_fps();
    double getSpeedAccuracy_fps();
    double getMotionHeading_deg();
    double getVehicleHeading_deg();
    double getHeadingAccuracy_deg();
    float getMagneticDeclination_deg();
    float getMagneticDeclinationAccuracy_deg();
    double getLongitude_rad();
    double getLatitude_rad();
    double getEllipsoidHeight_m();
    double getMSLHeight_m();
    double getHorizontalAccuracy_m();
    double getVerticalAccuracy_m();
    double getNorthVelocity_ms();
    double getEastVelocity_ms();
    double getDownVelocity_ms();
    double getGroundSpeed_ms();
    double getSpeedAccuracy_ms();
    double getMotionHeading_rad();
    double getVehicleHeading_rad();
    double getHeadingAccuracy_rad();
    float getMagneticDeclination_rad();
    float getMagneticDeclinationAccuracy_rad();
    float getpDOP();
    float gethDOP();
    float getvDOP();
    enum FixType getFixType();
    enum PowerSaveMode getPowerSaveMode();
    enum CarrierPhaseStatus getCarrierPhaseStatus();
    bool isGnssFixOk();
    bool isDiffCorrApplied();
    bool isHeadingValid();
    bool isConfirmedDate();
    bool isConfirmedTime();
    bool isTimeDateConfirmationAvail();
    bool isValidDate();
    bool isValidTime();
    bool isTimeFullyResolved();
    bool isMagneticDeclinationValid();
    String getNMEA_RMC();
    String getNMEA_GSA();
  private:

    enum frameComponent {
      Header = 0,
      Preambule,
      Payload,
      Chksum
    };
    HardwareSerial* _bus;
    uint32_t _baud;
    uint8_t _wr;
    frameComponent frameState;
    const uint8_t _ubxHeader[2] = {0xB5, 0x62};
    uint8_t _byte;
    struct _UBX_PREAMBULE {
      uint8_t msg_class;
      uint8_t msg_id;
      uint16_t msg_length;
    };

    struct _UBX_NAV_PVT_payload {
      uint32_t iTOW;
      uint16_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t min;
      uint8_t sec;
      uint8_t valid;
      uint32_t tAcc;
      int32_t nano;
      uint8_t fixType;
      uint8_t flags;
      uint8_t flags2;
      uint8_t numSV;
      int32_t lon;
      int32_t lat;
      int32_t height;
      int32_t hMSL;
      uint32_t hAcc;
      uint32_t vAcc;
      int32_t velN;
      int32_t velE;
      int32_t velD;
      int32_t gSpeed;
      int32_t headMot;
      uint32_t sAcc;
      uint32_t headAcc;
      uint16_t pDOP;
      uint8_t reserved[6];
      int32_t headVeh;
      int16_t magDec;
      uint16_t magAcc;
    };
    struct _UBX_NAV_SAT_svId_Item_flags{
      uint8_t qualityInd:3;
      uint8_t svUsed:1;
      uint8_t health:2;
      uint8_t diffCorr:1;
      uint8_t smoothed:1;
      uint8_t orbitSource:3;
      uint8_t ephAvail:1;
      uint8_t almAvail:1;
      uint8_t anoAvail:1;
      uint8_t aopAvail:1;
      uint8_t reserved1:1;
      uint8_t sbasCorrUsed:1;
      uint8_t rtcmCorrUsed:1;
      uint8_t reserved2:2;
      uint8_t prCorrUsed:1;
      uint8_t crCorrUsed:1;
      uint8_t doCorrUsed:1;
      uint8_t reserved3:9;
    };
    struct _UBX_NAV_SAT_svId_Item {
      uint8_t gnssId;
      uint8_t svId;
      uint8_t cno;
      int8_t  elev;
      int16_t azim;
      int16_t prRes;
      _UBX_NAV_SAT_svId_Item_flags flags;
    };
    struct _UBX_NAV_SAT_payload {
      uint32_t iTOW;
      uint8_t version;
      uint8_t numSvs;
      uint8_t reserved1;
    };
    struct _UBX_NAV_DOP_payload {
      uint32_t iTOW;
      uint16_t gDOP;
      uint16_t pDOP;
      uint16_t tDOP;
      uint16_t vDOP;
      uint16_t hDOP;
      uint16_t nDOP;
      uint16_t eDOP;
    };
    uint8_t staticRxBuff[MAX_FRAME_LENGTH];
    uint8_t *msg_preambule;
    uint8_t *msg_payload; // look for max payload length
    uint8_t *msg_chksum;
    struct _UBX_PREAMBULE* _validPreambule;
    struct _UBX_NAV_PVT_payload _validPVTPayload;
    struct _UBX_NAV_DOP_payload _validDOPPayload;
    struct _UBX_NAV_SAT_payload _validSATPayload;
    struct _UBX_NAV_SAT_svId_Item *_validSATsvItems;
    
    
    bool _parse(uint8_t msg_class,uint8_t msg_id,uint16_t msg_length);
    void _calcChecksum(uint8_t* CK, uint8_t* payload, int size );
    
    const double _PI = 3.14159265358979323846;
    const float _m2ft = 3.28084;
    const float _deg2rad = _PI/180.0;
};

#endif
 
