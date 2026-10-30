#pragma once
#include <stdint.h>
#include <winioctl.h>

// Private simulator device interface. It is deliberately NOT a COM interface.
// {8FDB2341-8F14-4C5C-A630-7EE2B852A417}
static const GUID KINISI_HOST_INTERFACE =
    {0x8fdb2341,0x8f14,0x4c5c,{0xa6,0x30,0x7e,0xe2,0xb8,0x52,0xa4,0x17}};
#define KINISI_HARDWARE_ID L"Kinisi\\VirtualBoard"
#define KINISI_HOST_REFERENCE L"host"
#define KINISI_PACKET_VERSION 1U
#define KINISI_PACKET_CAPACITY 4096U
#define IOCTL_KINISI_EXCHANGE CTL_CODE(FILE_DEVICE_UNKNOWN,0x801,METHOD_BUFFERED,FILE_READ_ACCESS|FILE_WRITE_ACCESS)
typedef struct KinisiPacket {
    uint32_t version;
    uint32_t generation;
    uint32_t length;
    uint32_t client_open;
    wchar_t port[16];
    uint8_t data[KINISI_PACKET_CAPACITY];
} KinisiPacket;
static_assert(sizeof(KinisiPacket)==4144,"Private driver ABI changed");
