#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <swdevice.h>
#include <cfgmgr32.h>
#include <sddl.h>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cstring>
#include "port_contract.h"
#include "../port.hpp"

namespace {
struct Creation {
    HANDLE event=CreateEventW(nullptr,TRUE,FALSE,nullptr);
    HRESULT result=E_PENDING;
    std::wstring instance;
    ~Creation(){if(event)CloseHandle(event);}
};
void WINAPI created(HSWDEVICE,HRESULT result,void* context,PCWSTR instance) {
    auto* state=static_cast<Creation*>(context);
    state->result=result;if(instance)state->instance=instance;SetEvent(state->event);
}
std::runtime_error failure(const char* what,DWORD code=GetLastError()) {
    return std::runtime_error(std::string(what)+" (Windows error "+std::to_string(code)+")");
}
PSECURITY_DESCRIPTOR owner_security() {
    HANDLE token;
    if(!OpenProcessToken(GetCurrentProcess(),TOKEN_QUERY,&token))throw failure("Open process token");
    DWORD needed=0;GetTokenInformation(token,TokenUser,nullptr,0,&needed);
    std::vector<uint8_t> buffer(needed);
    BOOL ok=GetTokenInformation(token,TokenUser,buffer.data(),needed,&needed);CloseHandle(token);
    if(!ok)throw failure("Read process identity");
    LPWSTR sid=nullptr;
    if(!ConvertSidToStringSidW(reinterpret_cast<TOKEN_USER*>(buffer.data())->User.Sid,&sid))throw failure("Format identity");
    std::wstring sddl=L"O:"+std::wstring(sid)+L"D:P(A;;GA;;;SY)(A;;GA;;;BA)(A;;GA;;;"+sid+L")";
    LocalFree(sid);
    PSECURITY_DESCRIPTOR security=nullptr;
    if(!ConvertStringSecurityDescriptorToSecurityDescriptorW(sddl.c_str(),SDDL_REVISION_1,&security,nullptr))throw failure("Device security");
    return security;
}
class WindowsPort final : public Port {
    HSWDEVICE device_=nullptr;
    HANDLE host_=INVALID_HANDLE_VALUE;
    uint32_t generation_=0;
    bool changed_=false;
    std::vector<uint8_t> incoming_;
    Creation creation_; // Lives until SwDeviceClose cancels/waits for callbacks.
    void exchange(const uint8_t* data=nullptr,size_t length=0) {
        KinisiPacket input{},output{};input.version=KINISI_PACKET_VERSION;
        input.generation=generation_;input.length=static_cast<uint32_t>(length);
        if(length)std::memcpy(input.data,data,length);
        DWORD returned=0;
        if(!DeviceIoControl(host_,IOCTL_KINISI_EXCHANGE,&input,sizeof(input),&output,sizeof(output),&returned,nullptr))
            throw failure("Simulator driver exchange");
        if(returned!=sizeof(output) || output.version!=KINISI_PACKET_VERSION || output.length>KINISI_PACKET_CAPACITY)
            throw std::runtime_error("Incompatible Kinisi driver response");
        if(output.generation!=generation_) {
            incoming_.clear();changed_=true;generation_=output.generation;
        }
        if(incoming_.size()+output.length>65536)throw std::runtime_error("Driver input queue overflow");
        incoming_.insert(incoming_.end(),output.data,output.data+output.length);
        if(name.empty()) {
            if(output.port[15]!=0)throw std::runtime_error("Invalid driver port name");
            for(auto c:output.port){if(!c)break;if(c>127)throw std::runtime_error("Invalid COM name");name.push_back(static_cast<char>(c));}
        }
    }
    void close() noexcept {
        if(host_!=INVALID_HANDLE_VALUE){CloseHandle(host_);host_=INVALID_HANDLE_VALUE;}
        if(device_){SwDeviceClose(device_);device_=nullptr;}
    }
public:
    WindowsPort() {
        try {
            if(!creation_.event)throw failure("Create device event");
            // Unique software instance permits independent boards; each owns one port.
            const auto instance=L"Board-"+std::to_wstring(GetCurrentProcessId());
            const wchar_t ids[]=KINISI_HARDWARE_ID L"\0";
            auto security=owner_security();
            SW_DEVICE_CREATE_INFO info{};info.cbSize=sizeof(info);info.pszInstanceId=instance.c_str();
            info.pszzHardwareIds=ids;info.pszDeviceDescription=L"Kinisi simulated controller";
            info.CapabilityFlags=SWDeviceCapabilitiesRemovable|SWDeviceCapabilitiesDriverRequired;
            info.pSecurityDescriptor=static_cast<SECURITY_DESCRIPTOR*>(security);
            HRESULT result=SwDeviceCreate(L"KinisiSimulator",L"HTREE\\ROOT\\0",&info,0,nullptr,created,&creation_,&device_);
            LocalFree(security);
            if(result==E_ACCESSDENIED)throw std::runtime_error("Run kinisi-sim from an Administrator terminal: Windows requires elevation to create a software device.");
            if(FAILED(result))throw failure("Create software device",static_cast<DWORD>(result));
            if(WaitForSingleObject(creation_.event,15000)!=WAIT_OBJECT_0)throw std::runtime_error("Software device creation timed out");
            if(FAILED(creation_.result))throw failure("Software device creation",static_cast<DWORD>(creation_.result));
            for(unsigned attempt=0;attempt<150;++attempt) {
                ULONG count=0;
                if(CM_Get_Device_Interface_List_SizeW(&count,const_cast<GUID*>(&KINISI_HOST_INTERFACE),
                    creation_.instance.data(),CM_GET_DEVICE_INTERFACE_LIST_PRESENT)==CR_SUCCESS && count>1) {
                    std::vector<wchar_t> paths(count);
                    if(CM_Get_Device_Interface_ListW(const_cast<GUID*>(&KINISI_HOST_INTERFACE),creation_.instance.data(),
                        paths.data(),count,CM_GET_DEVICE_INTERFACE_LIST_PRESENT)==CR_SUCCESS) {
                        host_=CreateFileW(paths.data(),GENERIC_READ|GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,
                                          nullptr,OPEN_EXISTING,0,nullptr);
                        if(host_!=INVALID_HANDLE_VALUE)break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if(host_==INVALID_HANDLE_VALUE)throw std::runtime_error(
                "Kinisi virtual COM driver is not installed or could not start. Install the signed native/windows/driver package; no COM pair is used.");
            exchange();changed_=false;
            if(name.empty())throw std::runtime_error("Driver did not publish a COM port name");
        }catch(...){close();throw;}
    }
    ~WindowsPort() override {close();}
    size_t read(uint8_t* buffer,size_t capacity) override {
        exchange();const auto count=std::min(capacity,incoming_.size());
        if(count)std::memcpy(buffer,incoming_.data(),count);
        incoming_.erase(incoming_.begin(),incoming_.begin()+count);return count;
    }
    size_t write(const uint8_t* buffer,size_t length) override {
        const auto count=std::min(length,size_t(KINISI_PACKET_CAPACITY));
        exchange(buffer,count);return count;
    }
    bool take_disconnect() override {bool result=changed_;changed_=false;return result;}
};
}
std::unique_ptr<Port> open_port(){return std::make_unique<WindowsPort>();}
