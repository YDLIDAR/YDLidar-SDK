#ifndef YDDATASTREAM_H
#define YDDATASTREAM_H
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <cstring>


class YdDataStream 
{
public:
    enum ByteOrder {
        LittleEndian,
        BigEndian
    };

    YdDataStream();
    explicit YdDataStream(ByteOrder order);
    explicit YdDataStream(
        const std::vector<uint8_t>& data, 
        ByteOrder order = LittleEndian);

    void setByteOrder(ByteOrder order);
    ByteOrder byteOrder() const;

    // ---------- 序列化 (<<) ----------
    YdDataStream& operator<<(uint8_t value);
    YdDataStream& operator<<(uint16_t value);
    YdDataStream& operator<<(uint32_t value);
    YdDataStream& operator<<(uint64_t value);
    YdDataStream& operator<<(int8_t value);
    YdDataStream& operator<<(int16_t value);
    YdDataStream& operator<<(int32_t value);
    YdDataStream& operator<<(int64_t value);
    YdDataStream& operator<<(float value);
    YdDataStream& operator<<(double value);

    // ---------- 反序列化 (>>) ----------
    YdDataStream& operator>>(uint8_t& value);
    YdDataStream& operator>>(uint16_t& value);
    YdDataStream& operator>>(uint32_t& value);
    YdDataStream& operator>>(uint64_t& value);
    YdDataStream& operator>>(int8_t& value);
    YdDataStream& operator>>(int16_t& value);
    YdDataStream& operator>>(int32_t& value);
    YdDataStream& operator>>(int64_t& value);
    YdDataStream& operator>>(float& value);
    YdDataStream& operator>>(double& value);

    // ---------- 原始字节块 ----------
    YdDataStream& writeBytes(const uint8_t* data, size_t size);
    YdDataStream& readBytes(uint8_t* dest, size_t size);

    // ---------- 流状态与定位 ----------
    bool atEnd() const;
    size_t readPos() const;
    void seekRead(size_t pos);
    void skipRead(size_t bytes);

    const std::vector<uint8_t>& data() const;
    void clear();

private:
    std::vector<uint8_t> m_buffer;
    size_t m_readPos;
    ByteOrder m_byteOrder;

    void checkReadBounds(size_t need) const;

    // 模板函数必须放在头文件（调用点需要看到定义）
    template<typename T>
    void writeInteger(T value) {
        constexpr size_t N = sizeof(T);
        uint8_t bytes[N];
        if (m_byteOrder == LittleEndian) {
            for (size_t i = 0; i < N; ++i)
                bytes[i] = static_cast<uint8_t>(value >> (i * 8));
        } else {
            for (size_t i = 0; i < N; ++i)
                bytes[N - 1 - i] = static_cast<uint8_t>(value >> (i * 8));
        }
        m_buffer.insert(m_buffer.end(), bytes, bytes + N);
    }

    template<typename T>
    void readInteger(T& value) {
        constexpr size_t N = sizeof(T);
        checkReadBounds(N);
        const uint8_t* src = m_buffer.data() + m_readPos;
        T tmp = 0;
        if (m_byteOrder == LittleEndian) {
            for (size_t i = 0; i < N; ++i)
                tmp |= static_cast<T>(src[i]) << (i * 8);
        } else {
            for (size_t i = 0; i < N; ++i)
                tmp = (tmp << 8) | static_cast<T>(src[i]);
        }
        value = tmp;
        m_readPos += N;
    }
};

#endif