#include "YdDataStream.h"

// ---------- 构造 ----------
YdDataStream::YdDataStream()
    : m_readPos(0), m_byteOrder(LittleEndian) {}

YdDataStream::YdDataStream(ByteOrder order)
    : m_readPos(0), m_byteOrder(order) {}

YdDataStream::YdDataStream(const std::vector<uint8_t>& data, ByteOrder order)
    : m_buffer(std::move(data)), m_readPos(0), m_byteOrder(order) {}

// ---------- 字节序 ----------
void YdDataStream::setByteOrder(ByteOrder order) { m_byteOrder = order; }
YdDataStream::ByteOrder YdDataStream::byteOrder() const { return m_byteOrder; }

// ---------- 序列化 ----------
YdDataStream& YdDataStream::operator<<(uint8_t value) {
    m_buffer.push_back(value);
    return *this;
}

YdDataStream& YdDataStream::operator<<(uint16_t value) { writeInteger(value); return *this; }
YdDataStream& YdDataStream::operator<<(uint32_t value) { writeInteger(value); return *this; }
YdDataStream& YdDataStream::operator<<(uint64_t value) { writeInteger(value); return *this; }

YdDataStream& YdDataStream::operator<<(int8_t value)  { return *this << static_cast<uint8_t>(value); }
YdDataStream& YdDataStream::operator<<(int16_t value) { return *this << static_cast<uint16_t>(value); }
YdDataStream& YdDataStream::operator<<(int32_t value) { return *this << static_cast<uint32_t>(value); }
YdDataStream& YdDataStream::operator<<(int64_t value) { return *this << static_cast<uint64_t>(value); }

YdDataStream& YdDataStream::operator<<(float value) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    uint32_t tmp;
    std::memcpy(&tmp, &value, sizeof(tmp));
    return *this << tmp;
}

YdDataStream& YdDataStream::operator<<(double value) {
    static_assert(sizeof(double) == 8, "double must be 8 bytes");
    uint64_t tmp;
    std::memcpy(&tmp, &value, sizeof(tmp));
    return *this << tmp;
}

// ---------- 反序列化 ----------
YdDataStream& YdDataStream::operator>>(uint8_t& value) {
    checkReadBounds(1);
    value = m_buffer[m_readPos++];
    return *this;
}

YdDataStream& YdDataStream::operator>>(uint16_t& value) { 
    readInteger(value); return *this; 
    }

YdDataStream& YdDataStream::operator>>(uint32_t& value) { 
    readInteger(value); return *this; 
    }

YdDataStream& YdDataStream::operator>>(uint64_t& value) { 
    readInteger(value); return *this; 
    }

YdDataStream& YdDataStream::operator>>(int8_t& value) {
    uint8_t tmp;
    *this >> tmp;
    value = static_cast<int8_t>(tmp);
    return *this;
}

YdDataStream& YdDataStream::operator>>(int16_t& value) {
    uint16_t tmp;
    *this >> tmp;
    std::memcpy(&value, &tmp, sizeof(value));
    return *this;
}

YdDataStream& YdDataStream::operator>>(int32_t& value) {
    uint32_t tmp;
    *this >> tmp;
    std::memcpy(&value, &tmp, sizeof(value));
    return *this;
}

YdDataStream& YdDataStream::operator>>(int64_t& value) {
    uint64_t tmp;
    *this >> tmp;
    std::memcpy(&value, &tmp, sizeof(value));
    return *this;
}

YdDataStream& YdDataStream::operator>>(float& value) {
    uint32_t tmp;
    *this >> tmp;
    std::memcpy(&value, &tmp, sizeof(value));
    return *this;
}

YdDataStream& YdDataStream::operator>>(double& value) {
    uint64_t tmp;
    *this >> tmp;
    std::memcpy(&value, &tmp, sizeof(value));
    return *this;
}

// ---------- 原始字节块 ----------
YdDataStream& YdDataStream::writeBytes(const uint8_t* data, size_t size) {
    m_buffer.insert(m_buffer.end(), data, data + size);
    return *this;
}

YdDataStream& YdDataStream::readBytes(uint8_t* dest, size_t size) {
    checkReadBounds(size);
    std::memcpy(dest, m_buffer.data() + m_readPos, size);
    m_readPos += size;
    return *this;
}

// ---------- 流状态与定位 ----------
bool YdDataStream::atEnd() const { return m_readPos >= m_buffer.size(); }
size_t YdDataStream::readPos() const { return m_readPos; }

void YdDataStream::seekRead(size_t pos) {
    if (pos > m_buffer.size())
        throw std::out_of_range("YdDataStream::seekRead out of range");
    m_readPos = pos;
}

void YdDataStream::skipRead(size_t bytes) { seekRead(m_readPos + bytes); }

const std::vector<uint8_t>& YdDataStream::data() const { return m_buffer; }

void YdDataStream::clear() {
    m_buffer.clear();
    m_readPos = 0;
}

// ---------- 私有辅助 ----------
void YdDataStream::checkReadBounds(size_t need) const {
    if (m_readPos + need > m_buffer.size())
        throw std::out_of_range("YdDataStream: read past end of buffer");
}