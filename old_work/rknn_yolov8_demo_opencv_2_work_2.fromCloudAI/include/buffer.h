#pragma once
#include <vector>
#include <cstddef>

template <typename T = uint8_t>
class Buffer {
public:
    Buffer() = default;
    explicit Buffer(std::size_t size) : data_(size) {}

    void resize(std::size_t size) { data_.resize(size); }
    T* data() { return data_.data(); }
    const T* data() const { return data_.data(); }
    std::size_t size() const { return data_.size(); }
    bool empty() const { return data_.empty(); }

private:
    std::vector<T> data_;
};
