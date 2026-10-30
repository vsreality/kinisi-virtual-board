// Kinisi single-COM UMDF 2 endpoint. Firmware executes in kinisi-sim.exe.
// No paired COM endpoint, socket, or serial proxy is involved.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define WIN32_NO_STATUS
#include <windows.h>
#undef WIN32_NO_STATUS
#pragma warning(push)
#pragma warning(disable:4324) // WDF headers intentionally align request parameters.
#include <wdf.h>
#pragma warning(pop)
#define _NTDEF_
// WDK serial IOCTL definitions overlap winioctl.h included by UMDF.
#pragma warning(push)
#pragma warning(disable:4005)
#include <ntddser.h>
#pragma warning(pop)
#include <ntstrsafe.h>
#include <new>
#include "../port_contract.h"
#include "../ring.hpp"
#include "../read_timeout.hpp"

static const GUID ComInterface={0x86e0d1e0,0x8089,0x11d0,{0x9c,0xe4,0x08,0x00,0x3e,0x30,0x1f,0x73}};
struct Device {
    Ring<65536> to_host,to_client;
    WDFQUEUE reads,waits;
    WDFWAITLOCK lock;
    WDFFILEOBJECT host,client;
    ULONG generation,mask,events;
    SERIAL_TIMEOUTS timeouts;
    SERIAL_BAUD_RATE baud;
    SERIAL_LINE_CONTROL line;
    SERIAL_HANDFLOW flow;
    SERIAL_CHARS chars;
    ULONG modem_control;
    wchar_t port[16];
};
struct File { BOOLEAN host; };
struct Read { ULONGLONG start,last; size_t done; SERIAL_TIMEOUTS timeouts; };
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(Device,Dev)
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(File,FileData)
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(Read,ReadData)
struct Guard {
    WDFWAITLOCK lock;
    explicit Guard(Device* d):lock(d->lock){WdfWaitLockAcquire(lock,nullptr);}
    ~Guard(){WdfWaitLockRelease(lock);}
};

EVT_WDF_DRIVER_DEVICE_ADD AddDevice;
EVT_WDF_DEVICE_FILE_CREATE Create;
EVT_WDF_FILE_CLEANUP Cleanup;
EVT_WDF_IO_QUEUE_IO_READ ReadRequest;
EVT_WDF_IO_QUEUE_IO_WRITE WriteRequest;
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL Control;
EVT_WDF_TIMER Timer;

static bool is_host(WDFREQUEST r){auto f=WdfRequestGetFileObject(r);return f && FileData(f)->host;}
static void complete(WDFREQUEST r,NTSTATUS status=STATUS_SUCCESS,ULONG_PTR size=0){WdfRequestCompleteWithInformation(r,status,size);}
template<class T> static T* input(WDFREQUEST r) {
    T* result=nullptr;
    return NT_SUCCESS(WdfRequestRetrieveInputBuffer(r,sizeof(T),reinterpret_cast<void**>(&result),nullptr))?result:nullptr;
}
template<class T> static void output(WDFREQUEST r,const T& value) {
    T* buffer=nullptr;
    auto status=WdfRequestRetrieveOutputBuffer(r,sizeof(T),reinterpret_cast<void**>(&buffer),nullptr);
    if(!NT_SUCCESS(status)){complete(r,status);return;}
    *buffer=value;complete(r,STATUS_SUCCESS,sizeof(T));
}
static void drain(WDFQUEUE queue,NTSTATUS status) {
    WDFREQUEST request;
    while(NT_SUCCESS(WdfIoQueueRetrieveNextRequest(queue,&request)))complete(request,status);
}
static void notify(Device* d,ULONG events) {
    d->events|=events&d->mask;
    if(!d->events)return;
    WDFREQUEST request;
    if(NT_SUCCESS(WdfIoQueueRetrieveNextRequest(d->waits,&request))){ULONG flags=d->events;d->events=0;output(request,flags);}
}
static bool service_read(Device* d,WDFREQUEST request) {
    Read* state=ReadData(request);
    uint8_t* buffer=nullptr;size_t capacity=0;
    auto status=WdfRequestRetrieveOutputBuffer(request,1,reinterpret_cast<void**>(&buffer),&capacity);
    if(!NT_SUCCESS(status)){complete(request,status);return true;}
    const ULONGLONG now=GetTickCount64();
    size_t received=d->to_client.pop(buffer+state->done,capacity-state->done);
    state->done+=received;if(received)state->last=now;
    const auto& t=state->timeouts;
    bool done=read_finished(state->done,capacity,now-state->start,now-state->last,
                           t.ReadIntervalTimeout,t.ReadTotalTimeoutMultiplier,t.ReadTotalTimeoutConstant);
    if(done){complete(request,STATUS_SUCCESS,state->done);return true;}
    // The request was retrieved from reads. Forwarding to the same queue is invalid.
    status=WdfRequestRequeue(request);
    if(!NT_SUCCESS(status)){complete(request,status);return true;}
    return false;
}
static void service_reads(Device* d) {
    ULONG count=0;WdfIoQueueGetState(d->reads,&count,nullptr);
    // An incomplete request is requeued at the head. Preserve FIFO and stop here.
    while(count--) {
        WDFREQUEST request;
        if(!NT_SUCCESS(WdfIoQueueRetrieveNextRequest(d->reads,&request)))break;
        if(!service_read(d,request))break;
    }
}
void Timer(WDFTIMER timer) {
    Device* d=Dev(static_cast<WDFDEVICE>(WdfTimerGetParentObject(timer)));
    Guard guard(d);
    service_reads(d);
    WdfTimerStart(timer,WDF_REL_TIMEOUT_IN_MS(1));
}
void Create(WDFDEVICE device,WDFREQUEST request,WDFFILEOBJECT file) {
    Device* d=Dev(device);const UNICODE_STRING* name=WdfFileObjectGetFileName(file);
    Guard guard(d);
    const wchar_t host_path[]=L"\\" KINISI_HOST_REFERENCE;
    bool host=name && name->Length==sizeof(host_path)-sizeof(wchar_t) &&
        !memcmp(name->Buffer,host_path,name->Length);
    if(name && name->Length && !host){complete(request,STATUS_OBJECT_NAME_NOT_FOUND);return;}
    if(host ? d->host!=nullptr : d->client!=nullptr){complete(request,STATUS_SHARING_VIOLATION);return;}
    if(!host && !d->host){complete(request,STATUS_DEVICE_NOT_CONNECTED);return;}
    FileData(file)->host=host;
    if(host)d->host=file;
    else {d->client=file;++d->generation;d->to_host.clear();d->to_client.clear();d->events=0;d->mask=0;}
    complete(request);
}
void Cleanup(WDFFILEOBJECT file) {
    Device* d=Dev(WdfFileObjectGetDevice(file));
    Guard guard(d);
    // A rejected create also receives cleanup. It must not disturb another owner.
    if(file!=d->host && file!=d->client)return;
    if(file==d->host)d->host=nullptr;
    else d->client=nullptr;
    ++d->generation;d->to_host.clear();d->to_client.clear();d->events=0;
    drain(d->reads,STATUS_CANCELLED);drain(d->waits,STATUS_CANCELLED);
}
void ReadRequest(WDFQUEUE queue,WDFREQUEST request,size_t length) {
    Device* d=Dev(WdfIoQueueGetDevice(queue));
    Guard guard(d);
    if(is_host(request)){complete(request,STATUS_INVALID_DEVICE_REQUEST);return;}
    if(!d->host){complete(request,STATUS_DEVICE_NOT_CONNECTED);return;}
    if(!length){complete(request);return;}
    if(length>65536){complete(request,STATUS_INVALID_BUFFER_SIZE);return;}
    ULONG count=0;WdfIoQueueGetState(d->reads,&count,nullptr);
    if(count>=64){complete(request,STATUS_DEVICE_BUSY);return;}
    WDF_OBJECT_ATTRIBUTES attr;WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attr,Read);
    Read* state;
    NTSTATUS status=WdfObjectAllocateContext(request,&attr,reinterpret_cast<void**>(&state));
    if(!NT_SUCCESS(status)){complete(request,status);return;}
    new(state) Read{};state->start=state->last=GetTickCount64();state->timeouts=d->timeouts;
    status=WdfRequestForwardToIoQueue(request,d->reads);
    if(!NT_SUCCESS(status)){complete(request,status);return;}
    service_reads(d);
}
void WriteRequest(WDFQUEUE queue,WDFREQUEST request,size_t length) {
    Device* d=Dev(WdfIoQueueGetDevice(queue));
    Guard guard(d);
    if(is_host(request)){complete(request,STATUS_INVALID_DEVICE_REQUEST);return;}
    if(!d->host){complete(request,STATUS_DEVICE_NOT_CONNECTED);return;}
    if(!length){complete(request);return;}
    uint8_t* buffer=nullptr;
    auto status=WdfRequestRetrieveInputBuffer(request,length,reinterpret_cast<void**>(&buffer),nullptr);
    if(!NT_SUCCESS(status)){complete(request,status);return;}
    if(!d->to_host.push(buffer,length)){complete(request,STATUS_DEVICE_BUSY);return;}
    complete(request,STATUS_SUCCESS,length);
}
void Control(WDFQUEUE queue,WDFREQUEST request,size_t,size_t,ULONG code) {
    Device* d=Dev(WdfIoQueueGetDevice(queue));
    Guard guard(d);
    if(code==IOCTL_KINISI_EXCHANGE) {
        if(!is_host(request)){complete(request,STATUS_ACCESS_DENIED);return;}
        KinisiPacket* in=input<KinisiPacket>(request);KinisiPacket* out=nullptr;
        auto status=WdfRequestRetrieveOutputBuffer(request,sizeof(KinisiPacket),reinterpret_cast<void**>(&out),nullptr);
        if(!in || !NT_SUCCESS(status)){complete(request,STATUS_BUFFER_TOO_SMALL);return;}
        // METHOD_BUFFERED buffers may alias. Consume input before writing output.
        if(in->version!=KINISI_PACKET_VERSION || in->length>KINISI_PACKET_CAPACITY){complete(request,STATUS_INVALID_PARAMETER);return;}
        if(in->generation==d->generation && d->client && in->length) {
            if(!d->to_client.push(in->data,in->length)){complete(request,STATUS_DEVICE_BUSY);return;}
            notify(d,SERIAL_EV_RXCHAR);service_reads(d);
        }
        *out={};out->version=KINISI_PACKET_VERSION;out->generation=d->generation;out->client_open=d->client!=nullptr;
        memcpy(out->port,d->port,sizeof(out->port));
        out->length=static_cast<uint32_t>(d->to_host.pop(out->data,sizeof(out->data)));
        if(out->length && !d->to_host.size)notify(d,SERIAL_EV_TXEMPTY);
        complete(request,STATUS_SUCCESS,sizeof(*out));return;
    }
    if(is_host(request)){complete(request,STATUS_INVALID_DEVICE_REQUEST);return;}
    switch(code) {
    case IOCTL_SERIAL_GET_BAUD_RATE:output(request,d->baud);return;
    case IOCTL_SERIAL_SET_BAUD_RATE: {
        auto v=input<SERIAL_BAUD_RATE>(request);if(!v || !v->BaudRate)break;d->baud=*v;complete(request);return;}
    case IOCTL_SERIAL_GET_LINE_CONTROL:output(request,d->line);return;
    case IOCTL_SERIAL_SET_LINE_CONTROL: {
        auto v=input<SERIAL_LINE_CONTROL>(request);if(!v || v->WordLength<5 || v->WordLength>8 || v->Parity>4 || v->StopBits>2)break;
        d->line=*v;complete(request);return;}
    case IOCTL_SERIAL_GET_TIMEOUTS:output(request,d->timeouts);return;
    case IOCTL_SERIAL_SET_TIMEOUTS: {
        auto v=input<SERIAL_TIMEOUTS>(request);if(!v)break;
        if(v->ReadIntervalTimeout==MAXDWORD && v->ReadTotalTimeoutMultiplier==MAXDWORD && v->ReadTotalTimeoutConstant==MAXDWORD)break;
        d->timeouts=*v;complete(request);return;}
    case IOCTL_SERIAL_GET_CHARS:output(request,d->chars);return;
    case IOCTL_SERIAL_SET_CHARS:{auto v=input<SERIAL_CHARS>(request);if(!v)break;d->chars=*v;complete(request);return;}
    case IOCTL_SERIAL_GET_HANDFLOW:output(request,d->flow);return;
    case IOCTL_SERIAL_SET_HANDFLOW:{auto v=input<SERIAL_HANDFLOW>(request);if(!v)break;d->flow=*v;complete(request);return;}
    case IOCTL_SERIAL_GET_MODEMSTATUS:{ULONG flags=d->host?(MS_CTS_ON|MS_DSR_ON|MS_RLSD_ON):0;output(request,flags);return;}
    case IOCTL_SERIAL_GET_DTRRTS:output(request,d->modem_control);return;
    case IOCTL_SERIAL_SET_DTR:d->modem_control|=SERIAL_DTR_STATE;complete(request);return;
    case IOCTL_SERIAL_CLR_DTR:d->modem_control&=~SERIAL_DTR_STATE;complete(request);return;
    case IOCTL_SERIAL_SET_RTS:d->modem_control|=SERIAL_RTS_STATE;complete(request);return;
    case IOCTL_SERIAL_CLR_RTS:d->modem_control&=~SERIAL_RTS_STATE;complete(request);return;
    case IOCTL_SERIAL_SET_BREAK_ON:case IOCTL_SERIAL_SET_BREAK_OFF:complete(request);return;
    case IOCTL_SERIAL_SET_QUEUE_SIZE:if(!input<SERIAL_QUEUE_SIZE>(request))break;complete(request);return;
    case IOCTL_SERIAL_GET_WAIT_MASK:output(request,d->mask);return;
    case IOCTL_SERIAL_SET_WAIT_MASK: {
        auto v=input<ULONG>(request);if(!v || (*v & ~(SERIAL_EV_RXCHAR|SERIAL_EV_TXEMPTY|SERIAL_EV_CTS|SERIAL_EV_DSR|SERIAL_EV_RLSD|SERIAL_EV_BREAK|SERIAL_EV_ERR)))break;
        d->mask=*v;d->events=0;WDFREQUEST waiter;
        while(NT_SUCCESS(WdfIoQueueRetrieveNextRequest(d->waits,&waiter)))output(waiter,ULONG(0));
        complete(request);return;}
    case IOCTL_SERIAL_WAIT_ON_MASK: {
        if(!d->mask){complete(request,STATUS_INVALID_PARAMETER);return;}
        ULONG count=0;WdfIoQueueGetState(d->waits,&count,nullptr);
        if(count){complete(request,STATUS_INVALID_PARAMETER);return;}
        ULONG* out=nullptr;auto status=WdfRequestRetrieveOutputBuffer(request,sizeof(ULONG),reinterpret_cast<void**>(&out),nullptr);
        if(!NT_SUCCESS(status)){complete(request,status);return;}
        if(d->events){ULONG flags=d->events;d->events=0;output(request,flags);return;}
        status=WdfRequestForwardToIoQueue(request,d->waits);if(!NT_SUCCESS(status))complete(request,status);return;}
    case IOCTL_SERIAL_PURGE: {
        auto v=input<ULONG>(request);if(!v || (*v & ~ULONG(SERIAL_PURGE_TXABORT|SERIAL_PURGE_RXABORT|SERIAL_PURGE_TXCLEAR|SERIAL_PURGE_RXCLEAR)))break;
        if(*v & SERIAL_PURGE_RXABORT)drain(d->reads,STATUS_CANCELLED);
        if(*v & SERIAL_PURGE_RXCLEAR)d->to_client.clear();
        if(*v & SERIAL_PURGE_TXCLEAR)d->to_host.clear();
        complete(request);return;}
    case IOCTL_SERIAL_GET_COMMSTATUS: {
        SERIAL_STATUS value{};value.AmountInInQueue=static_cast<ULONG>(d->to_client.size);
        value.AmountInOutQueue=static_cast<ULONG>(d->to_host.size);output(request,value);return;}
    case IOCTL_SERIAL_GET_PROPERTIES: {
        SERIAL_COMMPROP p{};p.PacketLength=sizeof(p);p.PacketVersion=2;p.ServiceMask=SERIAL_SP_SERIALCOMM;
        p.MaxTxQueue=p.MaxRxQueue=p.CurrentTxQueue=p.CurrentRxQueue=65536;p.MaxBaud=SERIAL_BAUD_USER;
        p.ProvSubType=SERIAL_SP_RS232;p.ProvCapabilities=SERIAL_PCF_TOTALTIMEOUTS|SERIAL_PCF_INTTIMEOUTS;
        p.SettableParams=SERIAL_SP_BAUD|SERIAL_SP_DATABITS|SERIAL_SP_STOPBITS|SERIAL_SP_PARITY;
        p.SettableBaud=SERIAL_BAUD_USER;p.SettableData=SERIAL_DATABITS_5|SERIAL_DATABITS_6|SERIAL_DATABITS_7|SERIAL_DATABITS_8;
        p.SettableStopParity=SERIAL_STOPBITS_10|SERIAL_STOPBITS_15|SERIAL_STOPBITS_20|SERIAL_PARITY_NONE|SERIAL_PARITY_ODD|SERIAL_PARITY_EVEN|SERIAL_PARITY_MARK|SERIAL_PARITY_SPACE;
        output(request,p);return;}
    default:complete(request,STATUS_NOT_SUPPORTED);return;
    }
    complete(request,STATUS_INVALID_PARAMETER);
}

NTSTATUS AddDevice(WDFDRIVER,PWDFDEVICE_INIT init) {
    WdfDeviceInitSetIoType(init,WdfDeviceIoBuffered);
    WDF_FILEOBJECT_CONFIG files;WDF_FILEOBJECT_CONFIG_INIT(&files,Create,WDF_NO_EVENT_CALLBACK,Cleanup);
    WDF_OBJECT_ATTRIBUTES fa;WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&fa,File);
    WdfDeviceInitSetFileObjectConfig(init,&files,&fa);
    WDF_OBJECT_ATTRIBUTES attr;WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attr,Device);
    attr.SynchronizationScope=WdfSynchronizationScopeNone;attr.ExecutionLevel=WdfExecutionLevelPassive;
    WDFDEVICE device;NTSTATUS status=WdfDeviceCreate(&init,&attr,&device);
    if(!NT_SUCCESS(status))return status;
    Device* d=Dev(device);new(d) Device{};d->baud.BaudRate=115200;d->line.WordLength=8;
    WDF_OBJECT_ATTRIBUTES la;WDF_OBJECT_ATTRIBUTES_INIT(&la);la.ParentObject=device;
    status=WdfWaitLockCreate(&la,&d->lock);if(!NT_SUCCESS(status))return status;
    WDF_IO_QUEUE_CONFIG q;WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&q,WdfIoQueueDispatchParallel);
    q.PowerManaged=WdfFalse;q.EvtIoRead=ReadRequest;q.EvtIoWrite=WriteRequest;q.EvtIoDeviceControl=Control;
    status=WdfIoQueueCreate(device,&q,WDF_NO_OBJECT_ATTRIBUTES,WDF_NO_HANDLE);if(!NT_SUCCESS(status))return status;
    WDF_IO_QUEUE_CONFIG_INIT(&q,WdfIoQueueDispatchManual);q.PowerManaged=WdfFalse;
    status=WdfIoQueueCreate(device,&q,WDF_NO_OBJECT_ATTRIBUTES,&d->reads);if(!NT_SUCCESS(status))return status;
    status=WdfIoQueueCreate(device,&q,WDF_NO_OBJECT_ATTRIBUTES,&d->waits);if(!NT_SUCCESS(status))return status;
    DECLARE_CONST_UNICODE_STRING(port_key,L"PortName");WDFKEY key;
    status=WdfDeviceOpenRegistryKey(device,PLUGPLAY_REGKEY_DEVICE,KEY_QUERY_VALUE,WDF_NO_OBJECT_ATTRIBUTES,&key);
    if(!NT_SUCCESS(status))return status;
    UNICODE_STRING port{};port.Buffer=d->port;port.MaximumLength=sizeof(d->port)-sizeof(wchar_t);
    status=WdfRegistryQueryUnicodeString(key,&port_key,nullptr,&port);WdfRegistryClose(key);
    if(!NT_SUCCESS(status))return status;
    // UMDF runs in a service session; SDK processes need the global DOS namespace.
    wchar_t link[64]{};status=RtlStringCchPrintfW(link,64,L"\\DosDevices\\Global\\%ws",d->port);if(!NT_SUCCESS(status))return status;
    UNICODE_STRING link_name;RtlInitUnicodeString(&link_name,link);
    status=WdfDeviceCreateSymbolicLink(device,&link_name);if(!NT_SUCCESS(status))return status;
    status=WdfDeviceCreateDeviceInterface(device,&ComInterface,nullptr);if(!NT_SUCCESS(status))return status;
    DECLARE_CONST_UNICODE_STRING(host_ref,KINISI_HOST_REFERENCE);
    status=WdfDeviceCreateDeviceInterface(device,&KINISI_HOST_INTERFACE,&host_ref);if(!NT_SUCCESS(status))return status;
    WDF_TIMER_CONFIG timer;WDF_TIMER_CONFIG_INIT(&timer,Timer);timer.AutomaticSerialization=FALSE;
    WDF_OBJECT_ATTRIBUTES ta;WDF_OBJECT_ATTRIBUTES_INIT(&ta);ta.ParentObject=device;
    WDFTIMER handle;status=WdfTimerCreate(&timer,&ta,&handle);if(!NT_SUCCESS(status))return status;
    WdfTimerStart(handle,WDF_REL_TIMEOUT_IN_MS(1));return STATUS_SUCCESS;
}
extern "C" NTSTATUS DriverEntry(PDRIVER_OBJECT object,PUNICODE_STRING path) {
    WDF_DRIVER_CONFIG config;WDF_DRIVER_CONFIG_INIT(&config,AddDevice);
    return WdfDriverCreate(object,path,WDF_NO_OBJECT_ATTRIBUTES,&config,WDF_NO_HANDLE);
}
