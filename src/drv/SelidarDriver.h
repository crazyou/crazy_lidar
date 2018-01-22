#ifndef _SELIDAR_DRIVER_H_
#define _SELIDAR_DRIVER_H_

#include "CrazylidarTypes.h"
#include "Serial.h"
#include "CrazylidarProtocol.h"
#include "Condition.h"
#include "Utils.h"
#include <boost/thread/thread.hpp>

namespace NS_Crazylidar
{
  
#define DEFAULT_TIMEOUT 2000
#define MAX_SCAN_NODES 2048
  
  class CrazylidarDriver
  {
  public:
    CrazylidarDriver ();
    virtual
    ~CrazylidarDriver ();

  public:
    virtual int
    connect (const char * port_path, unsigned int baudrate, unsigned int flag);
    virtual void
    disconnect ();
    virtual bool
    isConnected ();

    virtual int
    reset (unsigned int timeout = DEFAULT_TIMEOUT);
    virtual int
    stop (unsigned int timeout = DEFAULT_TIMEOUT);

    virtual int
    getHealth (CrazylidarHealth &health_info, unsigned int timeout =
    DEFAULT_TIMEOUT);
    virtual int
    getDeviceInfo (CrazylidarInfo &info, unsigned int timeout =
    DEFAULT_TIMEOUT);

    virtual int
    startScan (unsigned int timeout = DEFAULT_TIMEOUT);

    virtual int
    grabScanData (CrazylidarMeasurementNode * nodebuffer, size_t & count,
                  unsigned int timeout = DEFAULT_TIMEOUT);
    void acsendScanData(CrazylidarMeasurementNode * nodebuffer, size_t & count);
	 virtual int
	getSerialId ();
	virtual int
	startMotor();
virtual int
	stopMotor();
  protected:
    int
    sendCommand (unsigned char cmd);
    void
    disableDataGrabbing ();
    int
    waitResponseHeader (CrazylidarPacketHead* header, unsigned int timeout =
    DEFAULT_TIMEOUT);

    int
    waitScanData (unsigned short& flag, CrazylidarMeasurementNode* nodes,
                  size_t& node_count, unsigned int timeout = DEFAULT_TIMEOUT);

    int
    cacheScanData ();

    bool connected;
    bool scanning;

    boost::mutex rxtx_lock;
    NS_NaviCommon::Condition data_cond;
    Serial* rxtx;

    CrazylidarMeasurementNode cached_scan_node_buf[2048];
    size_t cached_scan_node_count;
    
    boost::thread cache_thread;
  };

}

#endif
