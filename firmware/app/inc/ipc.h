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

struct Reader {
    const uint8_t* data;
    size_t len;
    size_t pos = 0;

    uint8_t u8(){ return data[pos++]; }
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

struct TestFrame {
    uint8_t value_1;
    uint8_t value_2;

    static std::vector<uint8_t> encode(const struct TestFrame& f){
        std::vector<uint8_t> buf;
        ipc::put_u8(buf, f.value_1);
        ipc::put_u8(buf, f.value_2);

        return buf;
    }

    static TestFrame decode(const uint8_t* data, size_t length){
        ipc::Reader r{data, length};
        TestFrame frame{};
        frame.value_1 = r.u8();
        frame.value_2 = r.u8();
        return frame;
    }
};

} // IPC