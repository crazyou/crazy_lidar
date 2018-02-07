#ifndef _SELIDAR_PROTOCOL_H_
#define _SELIDAR_PROTOCOL_H_

namespace NS_Crazylidar
{
  
#define SELIDAR_CMD_SYNC_BYTE        0xA5
  
  typedef struct
  {
    unsigned char cmd_word :6;
    unsigned char error :1;
  } CrazylidarCommand;
  
  typedef struct
  {
    unsigned char sync_word;
    unsigned short length;
    CrazylidarCommand cmd;
    unsigned short payload_len;
  }__attribute__((packed)) CrazylidarPacketHead;
  
  enum CrazylidarReqType
  {
    StopReq = 0x02,
    ResetReq = 0x04,
    GetInfoReq = 0x08,
    GetHealthReq = 0x06,
    StartScanReq = 0x0C,
  };
  
  enum CrazylidarRepType
  {
    GetInfoRep = 0x09, GetHealthRep = 0x07, StartScanRep = 0x0D,
  };
  
#define SELIDAR_START_RANGES 1687
#define SELIDAR_MIDDLE_RANGES 2250
#define SELIDAR_END_RANGES 2813
  
  typedef struct
  {
    CrazylidarPacketHead head;
    unsigned char model;
    unsigned char fw_minor;
    unsigned char fw_major;
    unsigned char hw_id;
    unsigned char sn[16];
  }__attribute__((packed)) CrazylidarInfo;
  
  enum CrazylidarHealthStatus
  {
    StatusFine = 0, StatusWarning, StatusError,
  };
  
  typedef struct
  {
    CrazylidarPacketHead head;
    unsigned char status;
    unsigned short err_code;
  }__attribute__((packed)) CrazylidarHealth;
  
  typedef struct
  {
    unsigned short angle_scale_100;
    unsigned short distance_scale_1000;
  } CrazylidarMeasurementNode;

}

#endif
