#include <stdio.h>
#include <iostream>
#include "CrazylidarDriver.h"
#include "CrazylidarTypes.h"

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using std::cin;
using std::cout;
using std::endl;

namespace NS_Crazylidar
{
  
// Serial Driver Impl
  
  CrazylidarDriver::CrazylidarDriver ()
      : connected (false), scanning (false)
  {
    rxtx = new Serial ();
  }
  
  CrazylidarDriver::~CrazylidarDriver ()
  {
    // force disconnection
    disconnect ();
    
    delete rxtx;
  }
  
  int
  CrazylidarDriver::connect (const char * port_path, unsigned int baudrate,
                          unsigned int flag)
  {
    if (isConnected ())
    {
    	printf("isCONNECTED FAILED\n");
      return Denied;
     }
    
    if (!rxtx)
    {
    	printf("rxtx FAILED\n");	
      return Invalid;
    }
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      // establish the serial connection...
      if (!rxtx->bind (port_path, baudrate) || !rxtx->open ())
      {
      	printf("bind  FAILED\n");
        return Invalid;
      }
      
      rxtx->flush (0);
    }
    
    connected = true;
    
    return Success;
  }
  
  void
  CrazylidarDriver::disconnect ()
  {
    if (!connected)
      return;
   // stop ();
    rxtx->close ();
  }
  
  bool
  CrazylidarDriver::isConnected ()
  {
    return connected;
  }
  
  int
  CrazylidarDriver::sendCommand (unsigned char cmd)
  {
    unsigned char pkt_header[16] = { 0 };
    unsigned char pkg[128] = { 0 };
    int pkg_size = 0;
    CrazylidarPacketHead * header =
        reinterpret_cast<CrazylidarPacketHead *> (pkt_header);
    unsigned char checksum = 0;
    
    if (!connected)
      return Failure;
    
    header->sync_word = SELIDAR_CMD_SYNC_BYTE;
    header->cmd.error = 0;
    header->cmd.cmd_word = cmd;
    header->payload_len = 0;
    header->length = sizeof(CrazylidarPacketHead) + 1;
    
    for (size_t i = 0; i < sizeof(CrazylidarPacketHead); i++)
    {
      checksum ^= pkt_header[i];
    }
    
    memcpy (pkg, pkt_header, sizeof(CrazylidarPacketHead));
    pkg_size += sizeof(CrazylidarPacketHead);
    
    memcpy (pkg + pkg_size, &checksum, 1);
    pkg_size++;
    printf("[PKG]");
	for(int i=0;i<pkg_size;i++)
	{
		printf("%x ", pkg[i]);
	}
	printf("\n");
    rxtx->senddata (pkg, pkg_size);
    
    return Success;
  }
  
  int
  CrazylidarDriver::reset (unsigned int timeout)
  {
    int ans;
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      if (IS_FAIL(ans = sendCommand (ResetReq)))
      {
        return ans;
      }
    }
    
    return Success;
  }
  
  int
  CrazylidarDriver::stop (unsigned int timeout)
  {
    int ans;
    disableDataGrabbing ();
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      if (IS_FAIL(ans = sendCommand (StopReq)))
      {
        return ans;
      }
    }
    
    return Success;
  }
  
  void
  CrazylidarDriver::disableDataGrabbing ()
  {
    scanning = false;
    cache_thread.join ();
  }
  
  int
  CrazylidarDriver::waitResponseHeader (CrazylidarPacketHead* header,
                                     unsigned int timeout)
  {
    int recvPos = 0;
    unsigned int startTs = NS_NaviCommon::getMs ();
    unsigned char recvBuffer[sizeof(CrazylidarPacketHead)];
    unsigned char *headerBuffer = reinterpret_cast<unsigned char *> (header);
    unsigned int waitTime;
    
    while ((waitTime = NS_NaviCommon::getMs () - startTs) <= timeout)
    {
      size_t remainSize = sizeof(CrazylidarPacketHead) - recvPos;
      size_t recvSize;
      int ans = rxtx->waitfordata (remainSize, timeout - waitTime, &recvSize);
      if (ans == Serial::ANS_DEV_ERR)
      {
        return Failure;
      }
      else if (ans == Serial::ANS_TIMEOUT)
      {
        return Timeout;
      }
      
      if (recvSize > remainSize)
        recvSize = remainSize;
      
      rxtx->recvdata (recvBuffer, recvSize);
      
      for (size_t pos = 0; pos < recvSize; ++pos)
      {
        unsigned char currentByte = recvBuffer[pos];
        
        if (recvPos == 0)
        {
          if (currentByte != SELIDAR_CMD_SYNC_BYTE)
            continue;
        }
        headerBuffer[recvPos++] = currentByte;
        if (recvPos == sizeof(CrazylidarPacketHead))
        {
          return Success;
        }
      }
    }
    return Timeout;
  }
  
  int
  CrazylidarDriver::getHealth (CrazylidarHealth & health_info, unsigned int timeout)
  {
    int ans;
    
    if (!isConnected ())
      return Failure;
    
    disableDataGrabbing ();
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      if (IS_FAIL(ans = sendCommand (GetHealthReq)))
      {
        return ans;
      }
      
      CrazylidarPacketHead response_header;
      if (IS_FAIL(ans = waitResponseHeader (&response_header, timeout)))
      {
        return ans;
      }
      
      if (response_header.cmd.cmd_word != GetHealthRep)
      {
        return Invalid;
      }
      
      size_t data_size = sizeof(CrazylidarHealth) - sizeof(CrazylidarPacketHead) + 1;
      
      if (rxtx->waitfordata (data_size, timeout) != Serial::ANS_OK)
      {
        return Timeout;
      }
      
      unsigned char health_data[128] = { 0 };
      
      rxtx->recvdata (health_data, data_size);
      
      health_info.head = response_header;
      memcpy (
          reinterpret_cast<unsigned char *> (&health_info)
              + sizeof(CrazylidarPacketHead),
          health_data, data_size - 1);
      
      unsigned char checksum = 0;
      
      for (size_t i = 0; i < sizeof(CrazylidarHealth); i++)
      {
        checksum ^= *((unsigned char*) &health_info + i);
      }
      
      if (checksum != health_data[data_size - 1])
      {
        return BadCRC;
      }
      
    }
    
    return Success;
  }
  
  int
  CrazylidarDriver::getDeviceInfo (CrazylidarInfo & info, unsigned int timeout)
  {
    int ans;
    
    if (!isConnected ())
      return Failure;
    
    disableDataGrabbing ();
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      if (IS_FAIL(ans = sendCommand (GetInfoReq)))
      {
        return ans;
      }
      
      CrazylidarPacketHead response_header;
      if (IS_FAIL(ans = waitResponseHeader (&response_header, timeout)))
      {
        return ans;
      }
      
      if (response_header.cmd.cmd_word != GetInfoRep)
      {
        return Invalid;
      }
      
      size_t data_size = sizeof(CrazylidarInfo) - sizeof(CrazylidarPacketHead) + 1;
      
      if (rxtx->waitfordata (data_size, timeout) != Serial::ANS_OK)
      {
        return Timeout;
      }
      
      unsigned char info_data[128] = { 0 };
      
      rxtx->recvdata (info_data, data_size);
      
      info.head = response_header;
      memcpy (
          reinterpret_cast<unsigned char *> (&info) + sizeof(CrazylidarPacketHead),
          info_data, data_size - 1);
      
      unsigned char checksum = 0;
      
      for (size_t i = 0; i < sizeof(CrazylidarInfo); i++)
      {
        checksum ^= *((unsigned char*) &info + i);
      }
      
      if (checksum != info_data[data_size - 1])
      {
        return BadCRC;
      }
      
    }
    
    return Success;
  }
  
  int
  CrazylidarDriver::cacheScanData ()
  {
    CrazylidarMeasurementNode local_buf[360];
    size_t count = 360;
    CrazylidarMeasurementNode local_scan[MAX_SCAN_NODES];
    size_t scan_count = 0;

    size_t cached_count = 0;

    int ans;
    memset (local_scan, 0, sizeof(local_scan));
    
    bool got_start_range = false;

    //discard first packet
    unsigned short isstart = 0;
    while(!isstart)
    {
		if(IS_FAIL(ans = waitScanData(isstart, local_buf, count)))
		{
			if(ans != Timeout)
			{
				scanning = false;
				return Timeout;
			}
		}
    }


    while (scanning)
	{
		isstart = 0;
		if (IS_FAIL(ans = waitScanData (isstart, local_buf, count)))
		{
			if (ans != Timeout)
			{
				scanning = false;
				return Timeout;
			}
		}

		boost::mutex::scoped_lock auto_lock (rxtx_lock);

		if (isstart == 1)
		{
			cached_scan_node_count = cached_count;
			memcpy(cached_scan_node_buf, local_scan, cached_scan_node_count*sizeof(CrazylidarMeasurementNode));
			data_cond.set ();
			cached_count = 0;
		}
		if(cached_count >= 2048)
		{
			scanning = false;
			return Timeout;
		}
		for (int i = 0; i < count; i++)
		{
			local_scan[cached_count++] = local_buf[i];
			}

     
		}
		scanning = false;
		return Success;
  }
  
  int
  CrazylidarDriver::waitScanData (unsigned short& flag,
                               CrazylidarMeasurementNode* nodes,
                               size_t& node_count, unsigned int timeout)
  {
    int ans;
    unsigned short angle_range;
    unsigned char checksum = 0;
    
    // waiting for confirmation
    CrazylidarPacketHead response_header;
    if (IS_FAIL(ans = waitResponseHeader (&response_header, timeout)))
    {
      return ans;
    }
    
    if (response_header.cmd.cmd_word != StartScanRep)
    {
      return Invalid;
    }
   
    for (size_t i = 0; i < sizeof(CrazylidarPacketHead); i++)
    {
      checksum ^= *((unsigned char*) &response_header + i);
    }
    
    size_t data_size = response_header.length - sizeof(CrazylidarPacketHead);
    flag = response_header.payload_len>>15;
    if (rxtx->waitfordata (data_size, timeout) != Serial::ANS_OK)
    {
      return Timeout;
    }
    
    unsigned char scan_data[1024] = { 0 };
    
    rxtx->recvdata (scan_data, data_size);
    
    for (size_t i = 0; i < data_size - 1; i++)
    {
      checksum ^= scan_data[i];
    }
    
    if (checksum != scan_data[data_size - 1])
    {
      return BadCRC;
    }
    
    int data_pos = 0;
    
    memcpy (&angle_range, scan_data, sizeof(angle_range));
    data_pos += sizeof(angle_range);
    
    node_count = (data_size - 2 - 2 - 1) / 2;
    
    unsigned short start_angle;
    memcpy (&start_angle, scan_data + data_pos, sizeof(start_angle));
    data_pos += sizeof(start_angle);
    
    for (size_t i = 0; i < node_count; i++)
    {
      unsigned short distance;
      
      memcpy (&distance, scan_data + data_pos, sizeof(distance));
      data_pos += sizeof(distance);
      
      nodes[i].angle_scale_100 = start_angle + (i * angle_range) / node_count;
      nodes[i].distance_scale_1000 = distance;
    }
    
    return Success;
  }
  
  int
  CrazylidarDriver::startScan (unsigned int timeout)
  {
    int ans;
    if (!connected)
    {
      printf("connected failed\n");
      return Failure;
    }
    if (scanning)
    {
      printf("not scanning\n");
      return Denied;
    }
    
   // stop ();
    rxtx->flush(0);
    // have to slow down the speed of sending cmd, otherwise next cmd will be discard by radar
    NS_NaviCommon::delay (100);
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      scanning = true;
      cache_thread = boost::thread (
          boost::bind (&CrazylidarDriver::cacheScanData, this));
    }
    
    return Success;
  }
  
  int
  CrazylidarDriver::grabScanData (CrazylidarMeasurementNode * nodebuffer,
                               size_t & count, unsigned int timeout)
  {
    
    switch (data_cond.wait (timeout / 1000))
    {
      case NS_NaviCommon::Condition::COND_TIMEOUT:
        count = 0;
        return Timeout;
      case NS_NaviCommon::Condition::COND_OK:
      {
        if (cached_scan_node_count == 0)
          return Timeout; //consider as timeout
          
        boost::mutex::scoped_lock auto_lock (rxtx_lock);

        size_t size_to_copy = min(count, cached_scan_node_count);
        
        memcpy (nodebuffer, cached_scan_node_buf,
                size_to_copy * sizeof(CrazylidarMeasurementNode));
        count = size_to_copy;
        cached_scan_node_count = 0;
	
      }
        return Success;
        
      default:
        count = 0;
        return Failure;
    }
  }

    void CrazylidarDriver::acsendScanData(CrazylidarMeasurementNode * nodebuffer, size_t & count)
    {
	size_t i = 0;
	size_t j = 0;
	CrazylidarMeasurementNode tnode;

	for(i=0;i<count;i++)
	{
		if(nodebuffer[i].angle_scale_100>=36000)
			nodebuffer[i].angle_scale_100 -= 36000;
	}
	for(i=0;i<count-1;i++)
	{
	    j=i+1;
	    for(;j<count;j++)
	    {
		if(nodebuffer[j].angle_scale_100<nodebuffer[i].angle_scale_100)
		{
		    memcpy(&tnode, &nodebuffer[i], sizeof(CrazylidarMeasurementNode));
		    memcpy(&nodebuffer[i], &nodebuffer[j], sizeof(CrazylidarMeasurementNode));
		    memcpy(&nodebuffer[j], &tnode, sizeof(CrazylidarMeasurementNode));
		}
	    }
	}

    }

	int
  CrazylidarDriver::getSerialId ()
  {
	return rxtx->serial_fd;
  }
	int
	 CrazylidarDriver::startMotor()
	{
		rxtx->clearDTR();
		return Success;
	}

	int CrazylidarDriver::stopMotor()
	{
		rxtx->setDTR();
		return Success;
	}
}
