//#include <regex>
#include <mutex>

#include "Crazyflie.h"
#include "crtp.h"
#include "bootloader.h"

#include "Crazyradio.h"
#include "CrazyflieUSB.h"

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <cmath>
#include <inttypes.h>

#define MAX_RADIOS 16
#define MAX_USB     4

Crazyradio* g_crazyradios[MAX_RADIOS];
std::mutex g_radioMutex[MAX_RADIOS];

CrazyflieUSB* g_crazyflieUSB[MAX_USB];
std::mutex g_crazyflieusbMutex[MAX_USB];

Logger EmptyLogger;

Crazyflie::Crazyflie(
  const std::string& link_uri,
  Logger& logger)
  : m_radio(nullptr)
  , m_transport(nullptr)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
  , m_logTocEntries()
  , m_logBlockCb()
  , m_paramTocEntries()
  , m_paramValues()
  , m_emptyAckCallback(nullptr)
  , m_linkQualityCallback(nullptr)
  , m_consoleCallback(nullptr)
  , m_logger(logger)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%" SCNx64,
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
      if (!g_crazyradios[m_devId]) {
        g_crazyradios[m_devId] = new Crazyradio(m_devId);
        // g_crazyradios[m_devId]->setAckEnable(false);
        g_crazyradios[m_devId]->setAckEnable(true);
        g_crazyradios[m_devId]->setArc(0);
      }
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    success = std::sscanf(link_uri.c_str(), "usb://%d",
       &m_devId) == 1;

    if (m_devId >= MAX_USB) {
      throw std::runtime_error("This version does not support that many CFs over USB. Adjust MAX_USB and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_crazyflieusbMutex[m_devId]);
      if (!g_crazyflieUSB[m_devId]) {
        g_crazyflieUSB[m_devId] = new CrazyflieUSB(m_devId);
      }
    }

    m_transport = g_crazyflieUSB[m_devId];
  }

  if (!success) {
    throw std::runtime_error("Uri is not valid!");
  }
}

void Crazyflie::logReset()
{
  crtpLogResetRequest request;
  startBatchRequest();
  addRequest(request, 1);
  handleRequests();
}

void Crazyflie::sendSetpoint(
  float roll,
  float pitch,
  float yawrate,
  uint16_t thrust)
{
  crtpSetpointRequest request(roll, pitch, yawrate, thrust);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void Crazyflie::sendFullStateSetpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate)
{
  crtpFullStateSetpointRequest request(
    x, y, z,
    vx, vy, vz,
    ax, ay, az,
    qx, qy, qz, qw,
    rollRate, pitchRate, yawRate);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void Crazyflie::sendExternalPositionUpdate(
  float x,
  float y,
  float z)
{
  crtpExternalPositionUpdate position(x, y, z);
  sendPacket((const uint8_t*)&position, sizeof(position));
}

void Crazyflie::sendMultiPositionUpdate(uint8_t id,
  uint8_t seq0, int16_t x0, int16_t y0, int16_t z0,
  uint8_t seq1, int16_t x1, int16_t y1, int16_t z1,
  uint8_t seq2, int16_t x2, int16_t y2, int16_t z2,
  uint8_t seq3, int16_t x3, int16_t y3, int16_t z3)
{
  crtpBPositionUpdate position(id, seq0, x0, y0, z0, seq1, x1, y1, z1, seq2, x2, y2, z2, seq3, x3, y3, z3);
  sendPacket((const uint8_t*)&position, sizeof(position));
}

void Crazyflie::sendBPositionUpdate(uint8_t id,
  uint8_t seq0, int16_t x0, int16_t y0, int16_t z0,
  uint8_t seq1, int16_t x1, int16_t y1, int16_t z1,
  uint8_t seq2, int16_t x2, int16_t y2, int16_t z2,
  uint8_t seq3, int16_t x3, int16_t y3, int16_t z3)
{
  crtpBPositionUpdate position(id, seq0, x0, y0, z0, seq1, x1, y1, z1, seq2, x2, y2, z2, seq3, x3, y3, z3);
  sendBPacket((const uint8_t*)&position, sizeof(position));
}

void Crazyflie::sendMultiSetpoint(uint8_t seq, uint8_t mode,
  int16_t x0, int16_t y0, int16_t z0,
  int16_t x1, int16_t y1,
  int16_t x2, int16_t y2,
  int16_t x3, int16_t y3,
  int16_t x4, int16_t y4)
{
  crtpBroadcastSetpoint setpoint(seq, mode, x0, y0, z0, x1, y1, x2, y2, x3, y3, x4, y4); 
  sendPacket((const uint8_t*)&setpoint, sizeof(setpoint));
}

void Crazyflie::sendBroadcastSetpoint(uint8_t seq, uint8_t mode,
  int16_t x0, int16_t y0, int16_t z0,
  int16_t x1, int16_t y1,
  int16_t x2, int16_t y2,
  int16_t x3, int16_t y3,
  int16_t x4, int16_t y4)
{
  crtpBroadcastSetpoint setpoint(seq, mode, x0, y0, z0, x1, y1, x2, y2, x3, y3, x4, y4); 
  sendBPacket((const uint8_t*)&setpoint, sizeof(setpoint));
}

void Crazyflie::sendPing()
{
  uint8_t ping = 0xFF;
  sendPacket(&ping, sizeof(ping));
}

/**
 * Transmits any outgoing packets to the crazyflie.
 */
void Crazyflie::transmitPackets()
{
  if (!m_outgoing_packets.empty())
  {
    std::vector<crtpPacket_t>::iterator it;
    for (it = m_outgoing_packets.begin(); it != m_outgoing_packets.end(); it++)
    {
      sendPacket(it->raw, it->size+1);
    }
    m_outgoing_packets.clear();
  }
}

// https://forum.bitcraze.io/viewtopic.php?f=9&t=1488
void Crazyflie::reboot()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  sendPacketOrTimeout(reboot_init, sizeof(reboot_init));

  const uint8_t reboot_to_firmware[] = {0xFF, 0xFE, 0xF0, 0x01};
  sendPacketOrTimeout(reboot_to_firmware, sizeof(reboot_to_firmware));
}

uint64_t Crazyflie::rebootToBootloader()
{
  bootloaderResetInitRequest req(TargetNRF51);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false);
  const bootloaderResetInitResponse* response = getRequestResult<bootloaderResetInitResponse>(0);

  uint64_t result =
      ((uint64_t)response->addr[0] << 0)
    | ((uint64_t)response->addr[1] << 8)
    | ((uint64_t)response->addr[2] << 16)
    | ((uint64_t)response->addr[3] << 24)
    | ((uint64_t)0xb1 << 32);

  const uint8_t reboot_to_bootloader[] = {0xFF, 0xFE, 0xF0, 0x00};
  // for (size_t i = 0; i < 10; ++i) {
  //   if (sendPacket(reboot_to_bootloader, sizeof(reboot_to_bootloader))) {
  //     break;
  //   }
  // }
  sendPacketOrTimeout(reboot_to_bootloader, sizeof(reboot_to_bootloader));

  return result;
}

void Crazyflie::sysoff()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x02};
  sendPacketOrTimeout(shutdown, sizeof(shutdown));
}

void Crazyflie::trySysOff()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x02};
  for (size_t i = 0; i < 10; ++i) {
    if (sendPacket(shutdown, sizeof(shutdown))) {
      break;
    }
  }
}

void Crazyflie::alloff()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x01};
  sendPacketOrTimeout(shutdown, sizeof(shutdown));
}

void Crazyflie::syson()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x03};
  sendPacketOrTimeout(shutdown, sizeof(shutdown));
}

float Crazyflie::vbat()
{
  struct nrf51vbatResponse
  {
    uint8_t dummy1;
    uint8_t dummy2;
    uint8_t dummy3;
    float vbat;
  } __attribute__((packed));

  const uint8_t shutdown[] = {0xFF, 0xFE, 0x04};
  startBatchRequest();
  addRequest(shutdown, 2);
  handleRequests();
  return getRequestResult<nrf51vbatResponse>(0)->vbat;
}

void Crazyflie::writeFlash(
  BootloaderTarget target,
  const std::vector<uint8_t>& data)
{
  // Get info about the target
  bootloaderGetInfoRequest req(target);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false);
  const bootloaderGetInfoResponse* response = getRequestResult<bootloaderGetInfoResponse>(0);
  uint16_t pageSize = response->pageSize;
  uint16_t flashStart = response->flashStart;
  uint16_t nBuffPage = response->nBuffPage;
  // std::cout << "pageSize: " << pageSize
  //           << " nBuffPage: " << nBuffPage
  //           << " nFlashPage: " << response->nFlashPage
  //           << " flashStart: " << flashStart
  //           << " version: " << (int)response->version
  //           << std::endl;

  uint16_t numPages = ceil(data.size() / (float)pageSize);
  if (numPages + flashStart >= response->nFlashPage) {
    std::stringstream sstr;
    sstr << "Requested size too large!";
    throw std::runtime_error(sstr.str());
  }
  // std::cout << "numPages: " << numPages << std::endl;

  // write flash
  size_t offset = 0;
  uint16_t usedBuffers = 0;
  // startBatchRequest();
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      // std::cout << "request: " << page << " " << address << std::endl;
      bootloaderLoadBufferRequest req(target, usedBuffers, address);
      size_t requestedSize = std::min<size_t>(data.size() - offset, std::min<size_t>(25, pageSize - address));
      memcpy(req.data, &data[offset], requestedSize);
      // addRequest(req, 0);
      // for (size_t i = 0; i < 10; ++i)
      // std::cout << "request: " << req.page << " " << req.address << " " << requestedSize << std::endl;
      // for (size_t i = 0; i < 10; ++i) {

      auto start = std::chrono::system_clock::now();
      // while (true) {
        sendPacketOrTimeout((uint8_t*)&req, 7 + requestedSize);
      //   startBatchRequest();
      //   bootloaderReadBufferRequest req2(target, usedBuffers, address);
      //   addRequest(req2, 7);
      //   handleRequests(/*crtpMode=*/false);
      //   const bootloaderReadBufferResponse* response = getRequestResult<bootloaderReadBufferResponse>(0);
      //   if (memcmp(req.data, response->data, requestedSize) == 0) {
      //     break;
      //   }
      //   auto end = std::chrono::system_clock::now();
      //   std::chrono::duration<double> elapsedSeconds = end-start;
      //   if (elapsedSeconds.count() > 1.0) {
      //     throw std::runtime_error("timeout");
      //   }
      // }
      offset += requestedSize;
      if (offset >= data.size()) {
        break;
      }
    }
    ++usedBuffers;
    if (usedBuffers == nBuffPage
        || page == numPages + flashStart - 1) {


      // startBatchRequest();
      // for (uint16_t buf = 0; buf < usedBuffers; ++buf) {
      //   for (uint16_t address = 0; address < pageSize; address += 25) {
      //     // std::cout << "request: " << page << " " << address << std::endl;
      //     bootloaderReadBufferRequest req(target, buf, address);
      //     addRequest(req, 7);
      //   }
      // }
      // handleRequests(/*crtpMode=*/false);


      // upload all the buffers now
      // std::cout << "try to upload buffers!" << std::endl;
      // handleRequests(/*crtpMode=*/false);
      // std::cout << "buffers uploaded!" << std::endl;

      // std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write flash
      bootloaderWriteFlashRequest req(target, 0, page - usedBuffers + 1, usedBuffers);
      sendPacketOrTimeout((uint8_t*)&req, sizeof(req));

      auto start = std::chrono::system_clock::now();

      size_t tries = 0;
      while (true) {
        Crazyradio::Ack ack;
        bootloaderFlashStatusRequest statReq(target);
        sendPacket((const uint8_t*)&statReq, sizeof(statReq), ack);
        if (   ack.ack
            && ack.size == 5
            && memcmp(&req, ack.data, 3) == 0) {
          if (ack.data[3] != 1 || ack.data[4] != 0) {
            throw std::runtime_error("Error during flashing!");
          }
          break;
        }

        // std::cout << page - usedBuffers + 1 << "," << usedBuffers << std::endl;
        // startBatchRequest();
        // bootloaderWriteFlashRequest req(target, 0, page - usedBuffers + 1, usedBuffers);
        // addRequest(req, 3);
        // handleRequests(/*crtpMode=*/false);
        // const bootloaderWriteFlashResponse* response = getRequestResult<bootloaderWriteFlashResponse>(0);
        // if (response->done == 1 && response->error == 0) {
        //   break;
        // }
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        if (elapsedSeconds.count() > 0.5) {
          start = end;
          sendPacketOrTimeout((uint8_t*)&req, sizeof(req));
          ++tries;
          if (tries > 5) {
            throw std::runtime_error("timeout");
          }
        }
      }

      // std::cout << "Flashed: " << (page - flashStart) / (float)numPages * 100.0 << " %" << std::endl;

      // get ready to fill more buffers
      // if (page != numPages + flashStart - 1) {
      //   startBatchRequest();
      // }
      usedBuffers = 0;
    }
  }


}

void Crazyflie::readFlash(
  BootloaderTarget target,
  size_t size,
  std::vector<uint8_t>& data)
{
  // Get info about the target
  bootloaderGetInfoRequest req(target);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false);
  const bootloaderGetInfoResponse* response = getRequestResult<bootloaderGetInfoResponse>(0);
  uint16_t pageSize = response->pageSize;
  uint16_t flashStart = response->flashStart;
  // std::cout << "pageSize: " << pageSize
  //           << " nBuffPage: " << response->nBuffPage
  //           << " nFlashPage: " << response->nFlashPage
  //           << " flashStart: " << flashStart
  //           << " version: " << (int)response->version
  //           << std::endl;

  uint16_t numPages = ceil(size / (float)pageSize);
  if (numPages + flashStart >= response->nFlashPage) {
    std::stringstream sstr;
    sstr << "Requested size too large!";
    throw std::runtime_error(sstr.str());
  }
  // std::cout << "numPages: " << numPages << std::endl;

  // read flash
  size_t offset = 0;
  startBatchRequest();
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      // std::cout << "request: " << page << " " << address << std::endl;
      bootloaderReadFlashRequest req(target, page, address);
      addRequest(req, 7);
      size_t requestedSize = std::min(25, pageSize - address);
      offset += requestedSize;
      if (offset > size) {
        break;
      }
    }
  }
  handleRequests(/*crtpMode=*/false);

  // update output
  data.resize(size);
  size_t i = 0;
  offset = 0;
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      const bootloaderReadFlashResponse* response = getRequestResult<bootloaderReadFlashResponse>(i++);
      size_t requestedSize = std::min(25, pageSize - address);
      // std::cout << "offset: " << offset << " reqS: " << requestedSize;
      memcpy(&data[offset], response->data, std::min(size - offset, requestedSize));
      offset += requestedSize;
      if (offset > size) {
        break;
      }
    }
  }
}

void Crazyflie::requestLogToc(bool forceNoCache)
{
  uint16_t len;
  uint32_t crc;

  
    // std::cout << "Fall back to V1 param API" << std::endl;

    crtpLogGetInfoRequest infoRequest;
    startBatchRequest();
    addRequest(infoRequest, 1);
    handleRequests();
    len = getRequestResult<crtpLogGetInfoResponse>(0)->log_len;
    crc = getRequestResult<crtpLogGetInfoResponse>(0)->log_crc;
    // std::cout << len << std::endl;

  // check if it is in the cache
  std::string fileName = "log" + std::to_string(crc) + ".csv";
  std::ifstream infile(fileName);

  if (forceNoCache || !infile.good()) {
    m_logger.info("Log: " + std::to_string(len));

    // Request detailed information
    startBatchRequest();
    
      for (size_t i = 0; i < len; ++i) {
        crtpLogGetItemRequest itemRequest(i);
        addRequest(itemRequest, 2);
      }
    handleRequests();

    // Update internal structure with obtained data
    m_logTocEntries.resize(len);
    
      for (size_t i = 0; i < len; ++i) {
        auto response = getRequestResult<crtpLogGetItemResponse>(i);
        LogTocEntry& entry = m_logTocEntries[i];
        entry.id = i;
        entry.type = (LogType)response->type;
        entry.group = std::string(&response->text[0]);
        entry.name = std::string(&response->text[entry.group.size() + 1]);
      }

    // Write a cache file
    {
      // Atomic file write: write in temporary file first to avoid race conditions
      std::string fileNameTemp = fileName + ".tmp";
      std::ofstream output(fileNameTemp);
      output << "id,type,group,name" << std::endl;
      for (const auto& entry : m_logTocEntries) {
        output << std::to_string(entry.id) << ","
               << std::to_string(entry.type) << ","
               << entry.group << ","
               << entry.name << std::endl;
      }
      // change the filename
      rename(fileNameTemp.c_str(), fileName.c_str());
    }
  } else {
    m_logger.info("Found variables in cache.");
    m_logTocEntries.clear();
    std::string line, cell;
    std::getline(infile, line); // ignore header
    while (std::getline(infile, line)) {
      std::stringstream lineStream(line);
      m_logTocEntries.resize(m_logTocEntries.size() + 1);
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().id = std::stoi(cell);
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().type = (LogType)std::stoi(cell);
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().group = cell;
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().name = cell;
    }
  }
}

void Crazyflie::requestParamToc(bool forceNoCache)
{
  uint16_t numParam;
  uint32_t crc;
  // Find the number of parameters in TOC
  
    // std::cout << "Fall back to V1 param API" << std::endl;

    crtpParamTocGetInfoRequest infoRequest;
    startBatchRequest();
    addRequest(infoRequest, 1);
    handleRequests();
    numParam = getRequestResult<crtpParamTocGetInfoResponse>(0)->numParam;
    crc = getRequestResult<crtpParamTocGetInfoResponse>(0)->crc;

  // check if it is in the cache
  std::string fileName = "params" + std::to_string(crc) + ".csv";
  std::ifstream infile(fileName);

  if (forceNoCache || !infile.good()) {
    m_logger.info("Params: " + std::to_string(numParam));

    // Request detailed information and values
    startBatchRequest();
      for (uint16_t i = 0; i < numParam; ++i) {
        crtpParamTocGetItemRequest itemRequest(i);
        addRequest(itemRequest, 2);
        crtpParamReadRequest readRequest(i);
        addRequest(readRequest, 1);
      }
    handleRequests();
    // Update internal structure with obtained data
    m_paramTocEntries.resize(numParam);

      for (uint16_t i = 0; i < numParam; ++i) {
        auto r = getRequestResult<crtpParamTocGetItemResponse>(i*2+0);
        auto val = getRequestResult<crtpParamValueResponse>(i*2+1);

        ParamTocEntry& entry = m_paramTocEntries[i];
        entry.id = i;
        entry.type = (ParamType)(r->length | r-> type << 2 | r->sign << 3);
        entry.readonly = r->readonly;
        entry.group = std::string(&r->text[0]);
        entry.name = std::string(&r->text[entry.group.size() + 1]);

        ParamValue v;
        std::memcpy(&v, &val->valueFloat, 4);
        m_paramValues[i] = v;
      }

    // Write a cache file
    {
      // Atomic file write: write in temporary file first to avoid race conditions
      std::string fileNameTemp = fileName + ".tmp";
      std::ofstream output(fileNameTemp);
      output << "id,type,readonly,group,name" << std::endl;
      for (const auto& entry : m_paramTocEntries) {
        output << std::to_string(entry.id) << ","
               << std::to_string(entry.type) << ","
               << std::to_string(entry.readonly) << ","
               << entry.group << ","
               << entry.name << std::endl;
      }
      // change the filename
      rename(fileNameTemp.c_str(), fileName.c_str());
    }
  } else {
    m_logger.info("Found variables in cache.");
    m_paramTocEntries.clear();
    std::string line, cell;
    std::getline(infile, line); // ignore header
    while (std::getline(infile, line)) {
      std::stringstream lineStream(line);
      m_paramTocEntries.resize(m_paramTocEntries.size() + 1);
      std::getline(lineStream, cell, ',');
      m_paramTocEntries.back().id = std::stoi(cell);
      std::getline(lineStream, cell, ',');
      m_paramTocEntries.back().type = (ParamType)std::stoi(cell);
      std::getline(lineStream, cell, ',');
      m_paramTocEntries.back().readonly = std::stoi(cell);
      std::getline(lineStream, cell, ',');
      m_paramTocEntries.back().group = cell;
      std::getline(lineStream, cell, ',');
      m_paramTocEntries.back().name = cell;
    }

    // Request values
      startBatchRequest();
      for (size_t i = 0; i < numParam; ++i) {
        crtpParamReadRequest readRequest(i);
        addRequest(readRequest, 1);
      }
      handleRequests();
      for (size_t i = 0; i < numParam; ++i) {
        auto val = getRequestResult<crtpParamValueResponse>(i);
        ParamValue v;
        std::memcpy(&v, &val->valueFloat, 4);
        m_paramValues[i] = v;
      }
  }
}

void Crazyflie::setParam(uint8_t id, const ParamValue& value) {

  startBatchRequest();
  bool found = false;
  for (auto&& entry : m_paramTocEntries) {
    if (entry.id == id) {
      found = true;
      switch (entry.type) {
        case ParamTypeUint8:
          {
            crtpParamWriteRequest<uint8_t> request(id, value.valueUint8);
            addRequest(request, 1);
            break;
          }
        case ParamTypeInt8:
          {
            crtpParamWriteRequest<int8_t> request(id, value.valueInt8);
            addRequest(request, 1);
            break;
          }
        case ParamTypeUint16:
          {
            crtpParamWriteRequest<uint16_t> request(id, value.valueUint16);
            addRequest(request, 1);
            break;
          }
        case ParamTypeInt16:
          {
            crtpParamWriteRequest<int16_t> request(id, value.valueInt16);
            addRequest(request, 1);
            break;
          }
        case ParamTypeUint32:
          {
            crtpParamWriteRequest<uint32_t> request(id, value.valueUint32);
            addRequest(request, 1);
            break;
          }
        case ParamTypeInt32:
          {
            crtpParamWriteRequest<int32_t> request(id, value.valueInt32);
            addRequest(request, 1);
            break;
          }
        case ParamTypeFloat:
          {
            crtpParamWriteRequest<float> request(id, value.valueFloat);
            addRequest(request, 1);
            break;
          }
      }
    }
  }

  if (!found) {
    std::stringstream sstr;
    sstr << "Could not find parameter with id " << id;
    throw std::runtime_error(sstr.str());
  }
  handleRequests();

  m_paramValues[id] = value;
}

void Crazyflie::sendBPacket(
  const uint8_t* data,
  uint32_t length)
{
  std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
  if (m_radio->getAddress() != m_address) {
    m_radio->setAddress(m_address);
  }
  if (m_radio->getChannel() != m_channel) {
    m_radio->setChannel(m_channel);
  }
  if (m_radio->getDatarate() != m_datarate) {
    m_radio->setDatarate(m_datarate);
  }
  if (m_radio->getAckEnable()) {
    m_radio->setAckEnable(false);
  }
  m_radio->sendPacketNoAck(data, length);
}

bool Crazyflie::sendPacket(
  const uint8_t* data,
  uint32_t length,
  bool useSafeLink)
{
  Crazyradio::Ack ack;
  sendPacket(data, length, ack, useSafeLink);
  return ack.ack;
}

 void Crazyflie::sendPacketOrTimeout(
   const uint8_t* data,
   uint32_t length,
   bool useSafeLink,
   float timeout)
{
  auto start = std::chrono::system_clock::now();
  while (!sendPacket(data, length, useSafeLink)) {
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    if (elapsedSeconds.count() > timeout) {
      throw std::runtime_error("timeout");
    }
  }
}

void Crazyflie::sendPacket(
  const uint8_t* data,
  uint32_t length,
  Crazyradio::Ack& ack,
  bool useSafeLink)
{
  static uint32_t numPackets = 0;
  static uint32_t numAcks = 0;

  numPackets++;

  if (m_radio) {
    std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
    if (m_radio->getAddress() != m_address) {
      m_radio->setAddress(m_address);
    }
    if (m_radio->getChannel() != m_channel) {
      m_radio->setChannel(m_channel);
    }
    if (m_radio->getDatarate() != m_datarate) {
      m_radio->setDatarate(m_datarate);
    }
    if (!m_radio->getAckEnable()) {
      m_radio->setAckEnable(true);
    }
    // consider safelink here:
    //    Adds 1bit counter to CRTP header to guarantee that no ack (downlink)
    //    payload are lost and no uplink packet are duplicated.
    //    The caller should resend packet if not acked (ie. same as with a
    //    direct call to crazyradio.send_packet)
    if (useSafeLink) {
      std::vector<uint8_t> dataCopy(data, data + length);
      dataCopy[0] &= 0xF3;
      dataCopy[0] |= m_curr_up << 3 | m_curr_down << 2;
      m_radio->sendPacket(dataCopy.data(), length, ack);
      if (ack.ack && ack.size > 0 && (ack.data[0] & 0x04) == (m_curr_down << 2)) {
        m_curr_down = 1 - m_curr_down;
      }
      if (ack.ack) {
        m_curr_up = 1 - m_curr_up;
      }
    } else {
      m_radio->sendPacket(data, length, ack);
    }

  } else {
    std::unique_lock<std::mutex> mlock(g_crazyflieusbMutex[m_devId]);
    m_transport->sendPacket(data, length, ack);
  }
  ack.data[ack.size] = 0;
  if (ack.ack) {
    handleAck(ack);
    numAcks++;
  }
  if (numPackets == 100) {
    if (m_linkQualityCallback) {
      // We just take the ratio of sent vs. acked packets here
      // for a sliding window of 100 packets
      float linkQuality = numAcks / (float)numPackets;
      m_linkQualityCallback(linkQuality);
    }
    numPackets = 0;
    numAcks = 0;
  }
}

void Crazyflie::handleAck(
  const Crazyradio::Ack& result)
{
  if (crtpConsoleResponse::match(result)) {
    if (result.size > 0) {
      crtpConsoleResponse* r = (crtpConsoleResponse*)result.data;
      if (m_consoleCallback) {
        m_consoleCallback(r->text);
      }
    }
    // ROS_INFO("Console: %s", r->text);
  }
  else if (crtpLogGetInfoResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogGetItemResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogControlResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogDataResponse::match(result)) {
    crtpLogDataResponse* r = (crtpLogDataResponse*)result.data;
    auto iter = m_logBlockCb.find(r->blockId);
    if (iter != m_logBlockCb.end()) {
      iter->second(r, result.size - 5);
    }
    else {
      m_logger.warning("Received unrequested data for block: " + std::to_string((int)r->blockId));
    }
  }
  else if (crtpParamTocGetInfoResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpParamTocGetItemResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpParamValueResponse::match(result)) {
    // handled in batch system
  }
  else if (crtp(result.data[0]).port == 8) {
    // handled in batch system
  }
  else if (crtpPlatformRSSIAck::match(result)) {
    crtpPlatformRSSIAck* r = (crtpPlatformRSSIAck*)result.data;
    if (m_emptyAckCallback) {
      m_emptyAckCallback(r);
    }
  }
  else {
    crtp* header = (crtp*)result.data;
    m_logger.warning("Don't know ack: Port: " + std::to_string((int)header->port)
      + " Channel: " + std::to_string((int)header->channel)
      + " Len: " + std::to_string((int)result.size));
    // for (size_t i = 1; i < result.size; ++i) {
    //   std::cout << "    " << (int)result.data[i] << std::endl;
    // }
    queueGenericPacket(result);
  }
}

const Crazyflie::LogTocEntry* Crazyflie::getLogTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_logTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

const Crazyflie::ParamTocEntry* Crazyflie::getParamTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_paramTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

uint8_t Crazyflie::registerLogBlock(
  std::function<void(crtpLogDataResponse*, uint8_t)> cb)
{
  for (uint8_t id = 0; id < 255; ++id) {
    if (m_logBlockCb.find(id) == m_logBlockCb.end()) {
      m_logBlockCb[id] = cb;
      return id;
    }
  }
}

bool Crazyflie::unregisterLogBlock(
  uint8_t id)
{
  m_logBlockCb.erase(m_logBlockCb.find(id));
}

// Batch system

void Crazyflie::startBatchRequest()
{
  m_batchRequests.clear();
}

void Crazyflie::addRequest(
  const uint8_t* data,
  size_t numBytes,
  size_t numBytesToMatch)
{
  m_batchRequests.resize(m_batchRequests.size() + 1);
  m_batchRequests.back().request.resize(numBytes);
  memcpy(m_batchRequests.back().request.data(), data, numBytes);
  m_batchRequests.back().numBytesToMatch = numBytesToMatch;
  m_batchRequests.back().finished = false;
}

void Crazyflie::handleRequests(
  bool crtpMode,
  bool useSafeLink,
  float baseTime,
  float timePerRequest)
{
  auto start = std::chrono::system_clock::now();
  Crazyradio::Ack ack;
  m_numRequestsFinished = 0;
  bool sendPing = false;

  float timeout = baseTime + timePerRequest * m_batchRequests.size();

  while (true) {
    if (!crtpMode || !sendPing) {
      for (const auto& request : m_batchRequests) {
        if (!request.finished) {
          // std::cout << "sendReq" << std::endl;
          sendPacket(request.request.data(), request.request.size(), ack, useSafeLink);
          handleBatchAck(ack, crtpMode);

          auto end = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSeconds = end-start;
          if (elapsedSeconds.count() > timeout) {
            throw std::runtime_error("timeout");
          }
        }
      }
      sendPing = true;
    } else {
      for (size_t i = 0; i < 10; ++i) {
        uint8_t ping = 0xFF;
        sendPacket(&ping, sizeof(ping), ack, useSafeLink);
        handleBatchAck(ack, crtpMode);
        // if (ack.ack && crtpPlatformRSSIAck::match(ack)) {
        //   sendPing = false;
        // }

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        if (elapsedSeconds.count() > timeout) {
          throw std::runtime_error("timeout");
        }
      }

      sendPing = false;
    }
    if (m_numRequestsFinished == m_batchRequests.size()) {
      break;
    }
  }
}

void Crazyflie::handleBatchAck(
  const Crazyradio::Ack& ack,
  bool crtpMode)
{
  if (ack.ack) {
    for (auto& request : m_batchRequests) {
      if (crtpMode) {
        if ((crtp(ack.data[0]) == crtp(request.request[0]) || ack.data[0] == request.request[0])
            && memcmp(&ack.data[1], &request.request[1], request.numBytesToMatch) == 0
            && !request.finished) {
          request.ack = ack;
          request.finished = true;
          ++m_numRequestsFinished;
          // std::cout << "gotack" <<std::endl;
          return;
        }
      } else {
        if (!request.finished
            && memcmp(&ack.data[0], &request.request[0], request.numBytesToMatch) == 0) {
          request.ack = ack;
          request.finished = true;
          ++m_numRequestsFinished;
          // std::cout << m_numRequestsFinished / (float)m_batchRequests.size() * 100.0 << " %" << std::endl;
          return;
        }
      }
    }
    // ack is (also) handled in sendPacket
  }
}

