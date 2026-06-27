#pragma once

#include <cstdint>
#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

namespace ipc{

constexpr size_t CAN_FD_MAX_PAYLOAD = 64;

inline void put_u8(std::vector<std::uint8_t>&buf, uint8_t v){
    buf.push_back(v);
}

inline void put_i8(std::vector<std::uint8_t>&buf, int8_t v){
    buf.push_back(static_cast<uint8_t>(v));
}

inline void put_u16(std::vector<std::uint8_t>&buf, uint16_t v){
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

inline void put_i16(std::vector<std::uint8_t>&buf, int16_t v){
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

struct Reader {
    const uint8_t* data;
    size_t len;
    size_t pos = 0;

    uint8_t u8(){ return data[pos++]; }
    int8_t i8(){ return data[pos++]; }
    uint16_t u16() { 
        uint16_t v = static_cast<uint16_t>(data[pos] | data[pos + 1] << 8);
        pos += 2;
        return v;
    }
    int16_t i16() { 
        int16_t v = static_cast<int16_t>(data[pos] | data[pos + 1] << 8);
        pos += 2;
        return v;
    }
};


class IPC_Hook{
public:
    virtual ~IPC_Hook() = default;
    virtual void dispatch(const uint8_t* payload, size_t len) = 0;
    virtual uint8_t id() const = 0;
};

template <typename T>
class Hook : public IPC_Hook{
public:
    using Callback = std::function<void(const T&)>;

    Hook(uint8_t ipc_id, std::string name, Callback cb)
        : ipc_id_(ipc_id), name_(std::move(name)), cb_(std::move(cb)) {};

    void dispatch(const uint8_t* payload, size_t len) override {
        T obj = T::decode(payload, len);
        cb_(obj);
    }

    uint8_t id() const override { return ipc_id_; };

    std::vector<uint8_t> encode(const T& obj) const { return T::encode(obj); };

    const std::string& name() const { return name_; }

private:
    uint8_t ipc_id_;
    std::string name_;
    Callback cb_;

};

class IPCRouter{
    public:

    explicit IPCRouter(std::function<void(const uint8_t*, size_t)> tx_fn)
        : tx_fn_(std::move(tx_fn)) {}

    template <typename T>
    Hook<T>* register_hook(uint8_t ipc_id, const std::string& name,
        typename Hook<T>::Callback cb){
            auto hook = std::make_unique<Hook<T>>(ipc_id, name, std::move(cb));
            Hook<T>* ptr = hook.get();
            hooks_[ipc_id] = std::move(hook);
            return ptr;
        }

    void on_can_frame(const uint8_t* frame, size_t frame_len){
        if (frame_len < 2) return;
        
        uint8_t ipc_id = frame[0];
        dispatch(ipc_id, frame + 1, frame_len - 1);
    }

    void send(uint8_t ipc_id, const std::vector<uint8_t>& payload){
        std::vector<uint8_t> frame;
        frame.push_back(ipc_id);
        frame.insert(frame.end(), payload.begin(), payload.end());
        tx_fn_(frame.data(), frame.size());
    }

private:
    void dispatch(uint8_t ipc_id, const uint8_t* data, size_t len){
        auto it = hooks_.find(ipc_id);
        if (it != hooks_.end()) it->second->dispatch(data, len);
    }

    std::unordered_map<uint8_t, std::unique_ptr<IPC_Hook>> hooks_;
    std::function<void(const uint8_t*, size_t)> tx_fn_;
};

struct StateChange {
    uint8_t new_state;
    uint8_t old_state;
    static std::vector<uint8_t> encode(const struct StateChange& f){
        std::vector<uint8_t> buf;
        ipc::put_u8(buf, f.new_state);
        ipc::put_u8(buf, f.old_state);
        return buf;
    }

    static StateChange decode(const uint8_t* data, size_t length){
        ipc::Reader r{data, length};
        StateChange frame{};
        frame.new_state = r.u8();
        frame.old_state = r.u8();
        return frame;
    }
};

struct ControllerGains {
    uint16_t kP_mNm;
    uint16_t kD_mNm;

    static std::vector<uint8_t> encode(const struct ControllerGains& f){
        std::vector<uint8_t> buf;
        ipc::put_u16(buf, f.kP_mNm);
        ipc::put_u16(buf, f.kD_mNm);
        return buf;
    }

    static ControllerGains decode(const uint8_t* data, size_t length){
        ipc::Reader r{data, length};
        ControllerGains frame{};
        frame.kP_mNm = r.u16();
        frame.kD_mNm = r.u16();
        return frame;
    }
};

struct ControllerSetpoint {
    int16_t position_target_mRads;
    int16_t torque_feedforward_mNm;

    static std::vector<uint8_t> encode(const struct ControllerSetpoint& f){
        std::vector<uint8_t> buf;
        ipc::put_i16(buf, f.position_target_mRads);
        ipc::put_i16(buf, f.torque_feedforward_mNm);
        return buf;
    }

    static ControllerSetpoint decode(const uint8_t* data, size_t length){
        ipc::Reader r{data, length};
        ControllerSetpoint frame{};
        frame.position_target_mRads = r.i16();
        frame.torque_feedforward_mNm = r.i16();
        return frame;
    }
};

struct ControllerFeedback {
    int16_t position_mRads;
    int16_t velocity_mRads;
    int16_t torque_mNm;
    uint8_t drive_temp_C;

    static std::vector<uint8_t> encode(const struct ControllerFeedback& f){
        std::vector<uint8_t> buf;
        ipc::put_i16(buf, f.position_mRads);
        ipc::put_i16(buf, f.velocity_mRads);
        ipc::put_i16(buf, f.torque_mNm);
        ipc::put_u8(buf, f.drive_temp_C);
        return buf;
    }

    static ControllerFeedback decode(const uint8_t* data, size_t length){
        ipc::Reader r{data, length};
        ControllerFeedback frame{};
        frame.position_mRads = r.i16();
        frame.velocity_mRads = r.i16();
        frame.torque_mNm = r.i16();
        frame.drive_temp_C = r.u8();
        return frame;
    }
};


} // IPC