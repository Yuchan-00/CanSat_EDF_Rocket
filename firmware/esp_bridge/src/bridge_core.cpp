/* bridge_core.cpp - see bridge_core.h */
#include "bridge_core.h"

#include <string.h>

BridgeChunker::BridgeChunker(const BridgeFrameRule &rule, uint16_t cap, uint32_t idle_us, uint32_t frame_wait_us)
    : rule_(rule), cap_(cap == 0u ? 1u : (cap > BRIDGE_ESPNOW_MAX ? (uint16_t)BRIDGE_ESPNOW_MAX : cap)),
      idle_us_(idle_us), frame_wait_us_(frame_wait_us), n_(0), t_last_(0)
{
}

bool BridgeChunker::push(uint8_t b, uint32_t now_us)
{
    if (n_ >= cap_)
        return false;
    buf_[n_++] = b;
    t_last_ = now_us;
    return true;
}

uint16_t BridgeChunker::frame_bytes() const
{
    if (n_ < 3u || buf_[0] != rule_.sync0 || buf_[1] != rule_.sync1)
        return 0;
    if (buf_[2] < rule_.len_min || buf_[2] > rule_.len_max)
        return 0;
    return (uint16_t)(buf_[2] + rule_.overhead);
}

uint16_t BridgeChunker::due(uint32_t now_us) const
{
    uint16_t need;
    uint32_t quiet;
    if (n_ == 0u)
        return 0;
    need = frame_bytes();
    if (need > 0u && need <= cap_ && n_ >= need)
        return need;                                      /* a complete frame: at once, alone */
    if (n_ >= cap_)
        return n_;                                        /* full */
    quiet = now_us - t_last_;                             /* wrap-safe */
    if (quiet >= (need > 0u ? frame_wait_us_ : idle_us_))
        return n_;
    return 0;
}

uint16_t BridgeChunker::take(uint8_t *out, uint16_t n)
{
    if (n > n_)
        n = n_;
    memcpy(out, buf_, n);
    memmove(buf_, buf_ + n, (size_t)(n_ - n));
    n_ = (uint16_t)(n_ - n);
    return n;
}

bool ByteRing::put(const uint8_t *p, uint16_t n)
{
    uint16_t h = head_, i;
    if (n > space()) {
        dropped_packets_++;
        dropped_bytes_ += n;
        return false;
    }
    for (i = 0; i < n; i++) {
        buf_[h] = p[i];
        h = (uint16_t)((h + 1u) & (SIZE - 1u));
    }
    head_ = h;
    return true;
}

uint16_t ByteRing::get(uint8_t *out, uint16_t max)
{
    uint16_t n = 0, t = tail_;
    const uint16_t h = head_;
    while (t != h && n < max) {
        out[n++] = buf_[t];
        t = (uint16_t)((t + 1u) & (SIZE - 1u));
    }
    tail_ = t;
    return n;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

bool bridge_parse_mac(const char *s, uint8_t mac[6])
{
    int i;
    for (i = 0; i < 6; i++) {
        const int hi = hexval(s[0]), lo = hi < 0 ? -1 : hexval(s[1]);
        if (hi < 0 || lo < 0)
            return false;
        mac[i] = (uint8_t)(hi * 16 + lo);
        s += 2;
        if (i < 5) {
            if (*s != ':')
                return false;
            s++;
        }
    }
    return *s == '\0';
}
