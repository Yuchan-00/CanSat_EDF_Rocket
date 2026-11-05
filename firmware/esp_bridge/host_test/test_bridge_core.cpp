/* test_bridge_core.cpp - host test of src/bridge_core.* (the portable part of the ESP-NOW bridge).
 * Built and run by tools/test_esp_bridge.py with the venv's zig c++ (python -m ziglang c++).  Prints one line per failed
 * check and "BRIDGE_CORE <checks> checks, <failures> failures"; exit status 0 only without failures.
 *
 * What it proves (bridge_core.h invariant): for random byte streams with random arrival gaps, random poll times and
 * frames among garbage, the packets the chunker emits concatenate to the input stream, each holds 1..cap bytes, a
 * complete frame at the buffer start leaves alone and at once, nothing leaves before its idle / frame-wait time unless
 * the buffer is full, also for bursts pushed with no poll between them (review LB-1); the ring keeps packets whole and
 * in order, also at the exactly-full boundary (review LB-2); the MAC parser accepts exactly xx:xx:xx:xx:xx:xx. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <vector>

#include "../src/bridge_core.h"

static int g_checks, g_fail;

#define CHECK(c, ...)                                                                                                 \
    do {                                                                                                              \
        g_checks++;                                                                                                   \
        if (!(c)) {                                                                                                   \
            g_fail++;                                                                                                 \
            if (g_fail <= 40) {                                                                                       \
                printf("FAIL %s:%d: ", __FILE__, __LINE__);                                                           \
                printf(__VA_ARGS__);                                                                                  \
                printf("\n");                                                                                         \
            }                                                                                                         \
        }                                                                                                             \
    } while (0)

static uint32_t g_rng = 0x5A5A1234u;
static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}

static const BridgeFrameRule kRule = {0xA5, 0x5A, 3, 42, 5};   /* sim/link_proto.py (LEN_MAX 42, overhead 5) */
static const uint32_t IDLE = 781, WAIT = 13021;                 /* 3 byte times / 47 + 3 byte times at 38400 baud */

static std::vector<uint8_t> frame(uint8_t len)
{
    std::vector<uint8_t> f;
    f.push_back(0xA5);
    f.push_back(0x5A);
    f.push_back(len);
    for (uint8_t i = 0; i < len + 2u; i++)
        f.push_back((uint8_t)rnd());
    return f;
}

static void test_basic(void)
{
    BridgeChunker c(kRule, 250, IDLE, WAIT);
    uint8_t out[250];
    std::vector<uint8_t> f = frame(10);                         /* a CMD frame: 15 bytes */
    uint32_t t = 1000;
    for (size_t i = 0; i < f.size(); i++) {
        CHECK(c.due(t) == 0, "frame byte %u: nothing due before the frame is complete", (unsigned)i);
        CHECK(c.push(f[i], t), "push");
        t += 260;
    }
    CHECK(c.due(t) == f.size(), "complete frame due at once (%u)", c.due(t));
    CHECK(c.take(out, (uint16_t)f.size()) == f.size() && memcmp(out, f.data(), f.size()) == 0, "frame bytes unchanged");
    CHECK(c.pending() == 0, "empty after take");
    /* garbage: due only after IDLE */
    c.push(0x11, t);
    c.push(0x22, t + 260);
    CHECK(c.due(t + 260 + IDLE - 1) == 0, "garbage not due before idle");
    CHECK(c.due(t + 260 + IDLE) == 2, "garbage due at idle");
    c.take(out, 2);
    CHECK(out[0] == 0x11 && out[1] == 0x22, "garbage unchanged");
    /* an incomplete frame waits WAIT, not IDLE */
    t = 50000;
    c.push(0xA5, t);
    c.push(0x5A, t);
    c.push(30, t);
    c.push(0x01, t);
    CHECK(c.due(t + IDLE) == 0, "incomplete frame not flushed at idle");
    CHECK(c.due(t + WAIT) == 4, "incomplete frame flushed at the frame wait");
    c.take(out, 4);
    /* a frame followed by bytes: the frame alone first */
    f = frame(42);
    for (size_t i = 0; i < f.size(); i++)
        c.push(f[i], 90000);
    c.push(0x77, 90000);
    CHECK(c.due(90000) == f.size(), "frame first, alone (%u of %u)", c.due(90000), (unsigned)c.pending());
    c.take(out, c.due(90000));
    CHECK(c.pending() == 1 && c.due(90000) == 0 && c.due(90000 + IDLE) == 1, "the trailing byte waits for idle");
    c.take(out, 1);
    /* full buffer: due at once, push refuses the next byte */
    for (int i = 0; i < 250; i++)
        CHECK(c.push(0x00, 100), "fill %d", i);
    CHECK(!c.push(0x00, 100), "251st byte refused");
    CHECK(c.due(100) == 250, "full buffer due at once");
    c.take(out, 250);
    /* wrap of the microsecond clock */
    c.push(0x33, 0xFFFFFF00u);
    CHECK(c.due(0xFFFFFF00u + IDLE - 1u) == 0 && c.due((uint32_t)(0xFFFFFF00u + IDLE)) == 1, "idle across the uint32 wrap");
    c.take(out, 1);
    /* cap clamp: 0 -> 1, > 250 -> 250 */
    BridgeChunker c1(kRule, 0, IDLE, WAIT), c2(kRule, 1000, IDLE, WAIT);
    CHECK(c1.capacity() == 1 && c2.capacity() == 250, "cap clamp");
    /* a small cap: a frame longer than the cap is sent in pieces when the buffer is full */
    BridgeChunker c3(kRule, 16, IDLE, WAIT);
    f = frame(30);
    size_t got = 0;
    std::vector<uint8_t> all;
    for (size_t i = 0; i < f.size(); i++) {
        if (!c3.push(f[i], 7)) {
            uint16_t n = c3.due(7);
            CHECK(n == 16, "small cap: full -> 16 (%u)", n);
            c3.take(out, n);
            all.insert(all.end(), out, out + n);
            c3.push(f[i], 7);
        }
    }
    got = c3.due(7 + WAIT);
    c3.take(out, (uint16_t)got);
    all.insert(all.end(), out, out + got);
    CHECK(all == f, "small cap: the pieces concatenate to the frame");
    /* review LB-1: what loop() makes - several frames / garbage pushed before one due(): take() must keep the rest intact */
    {
        BridgeChunker c4(kRule, 250, IDLE, WAIT);
        const std::vector<uint8_t> f1 = frame(10), f2 = frame(42), f3 = frame(3);
        const uint8_t junk[5] = {0x11, 0xA5, 0x22, 0x5A, 0x33};
        std::vector<uint8_t> stream(f1);
        stream.insert(stream.end(), f2.begin(), f2.end());
        stream.insert(stream.end(), f3.begin(), f3.end());
        stream.insert(stream.end(), junk, junk + 5);
        for (size_t i = 0; i < stream.size(); i++)
            CHECK(c4.push(stream[i], 200000), "batch push %u", (unsigned)i);
        CHECK(c4.due(200000) == f1.size() && c4.take(out, (uint16_t)f1.size()) == f1.size()
              && memcmp(out, f1.data(), f1.size()) == 0, "frame + frame: the first alone, unchanged");
        CHECK(c4.due(200000) == f2.size() && c4.take(out, (uint16_t)f2.size()) == f2.size()
              && memcmp(out, f2.data(), f2.size()) == 0, "frame + frame: the second alone, unchanged");
        CHECK(c4.due(200000) == f3.size() && c4.take(out, (uint16_t)f3.size()) == f3.size()
              && memcmp(out, f3.data(), f3.size()) == 0, "frame + frame + frame: the third alone, unchanged");
        CHECK(c4.pending() == 5 && c4.due(200000) == 0, "frame + 5 garbage bytes: the garbage waits for idle");
        CHECK(c4.due(200000 + IDLE) == 5 && c4.take(out, 5) == 5 && memcmp(out, junk, 5) == 0,
              "frame + 5 garbage bytes: the garbage unchanged after idle");
    }
}

/* review LB-1: bursts - pieces pushed with no due() between them (loop() reads every UART byte first), then drained the
 * way loop() drains (due / take until nothing is due) */
static void test_batch(void)
{
    uint8_t out[250];
    for (int run = 0; run < 400; run++) {
        const uint16_t cap = (uint16_t)(run % 4 == 0 ? 8 + rnd() % 60 : 250);
        BridgeChunker c(kRule, cap, IDLE, WAIT);
        std::vector<uint8_t> in, outs;
        uint32_t t = rnd();
        for (int k = 0; k < 40; k++) {
            const int pieces = 1 + (int)(rnd() % 4);
            for (int p = 0; p < pieces; p++) {
                std::vector<uint8_t> piece;
                const uint32_t kind = rnd() % 3;
                if (kind == 0)
                    piece = frame((uint8_t)(3 + rnd() % 40));
                else {
                    piece.resize(1 + rnd() % (kind == 1 ? 5 : 30));
                    for (size_t i = 0; i < piece.size(); i++)
                        piece[i] = (uint8_t)rnd();
                }
                for (size_t i = 0; i < piece.size(); i++) {
                    t += 26;                                     /* the bytes of one UART read: no poll between them */
                    if (!c.push(piece[i], t)) {
                        const uint16_t n = c.due(t);
                        CHECK(n > 0 && n <= cap, "batch: refused push with nothing due (%u / %u)", n, cap);
                        c.take(out, n);
                        outs.insert(outs.end(), out, out + n);
                        CHECK(c.push(piece[i], t), "batch: push after take");
                    }
                    in.push_back(piece[i]);
                }
            }
            /* drain at one poll time, as loop(): due / take until nothing is due */
            const uint32_t tp = t + (rnd() % 2 ? 0u : rnd() % 20000);
            for (;;) {
                const std::vector<uint8_t> pend(in.begin() + (long)outs.size(), in.end());
                const uint16_t n = c.due(tp);
                uint16_t need = 0;
                CHECK(pend.size() == c.pending(), "batch: pending mirror");
                if (pend.size() >= 3 && pend[0] == 0xA5 && pend[1] == 0x5A && pend[2] >= 3 && pend[2] <= 42)
                    need = (uint16_t)(pend[2] + 5);
                const bool frame_now = need > 0 && need <= cap && pend.size() >= need;
                const bool full_now = pend.size() >= cap;
                const bool idle_now = !pend.empty() && tp - t >= (need ? WAIT : IDLE);
                if (!n) {
                    CHECK(!frame_now && !full_now && !idle_now, "batch: held back (pending %u, need %u)",
                          (unsigned)pend.size(), need);
                    break;
                }
                CHECK(frame_now ? n == need : ((full_now || idle_now) && n == pend.size()),
                      "batch: flush rule (n %u, pending %u, need %u)", n, (unsigned)pend.size(), need);
                c.take(out, n);
                CHECK(memcmp(out, pend.data(), n) == 0, "batch: packet = the pending bytes' head");
                outs.insert(outs.end(), out, out + n);
            }
            t = tp;
        }
        uint16_t n;
        while ((n = c.due(t + WAIT + 1)) > 0) {
            c.take(out, n);
            outs.insert(outs.end(), out, out + n);
        }
        CHECK(c.pending() == 0 && outs == in, "batch run %d: packets concatenate to the input (%u vs %u bytes)", run,
              (unsigned)outs.size(), (unsigned)in.size());
    }
}

/* random streams: the transparency invariant and the timing rule */
static void test_random(void)
{
    uint8_t out[250];
    for (int run = 0; run < 400; run++) {
        const uint16_t cap = (uint16_t)(run % 5 == 0 ? 8 + rnd() % 40 : 250);
        BridgeChunker c(kRule, cap, IDLE, WAIT);
        std::vector<uint8_t> in, outs;
        uint32_t t = rnd(), t_last = t;
        int packets = 0;
        for (int k = 0; k < 60; k++) {
            std::vector<uint8_t> piece;
            const uint32_t kind = rnd() % 4;
            if (kind == 0)
                piece = frame((uint8_t)(3 + rnd() % 40));
            else if (kind == 1)
                piece.assign(1 + rnd() % 6, 0xA5);                /* sync-like garbage */
            else {
                piece.resize(1 + rnd() % 30);
                for (size_t i = 0; i < piece.size(); i++)
                    piece[i] = (uint8_t)rnd();
            }
            for (size_t i = 0; i < piece.size(); i++) {
                const uint32_t gap = rnd() % 8 == 0 ? rnd() % 20000 : 260;
                /* poll at increasing random times inside the gap (before the next byte arrives), as loop() does */
                uint32_t tp = t_last;
                for (int q = 0; q < 3 && gap > 0; q++) {
                    tp += rnd() % (gap - (tp - t_last) + 1u);
                    if (tp - t_last >= gap)
                        break;
                    const uint16_t n = c.due(tp);
                    /* the rule, from the pending bytes (input not yet emitted) */
                    const std::vector<uint8_t> pend(in.begin() + (long)outs.size(), in.end());
                    uint16_t need = 0;
                    if (pend.size() >= 3 && pend[0] == 0xA5 && pend[1] == 0x5A && pend[2] >= 3 && pend[2] <= 42)
                        need = (uint16_t)(pend[2] + 5);
                    const uint32_t quiet = tp - t_last;
                    const bool frame_now = need > 0 && need <= cap && pend.size() >= need;
                    const bool full_now = pend.size() >= cap;
                    const bool idle_now = !pend.empty() && quiet >= (need ? WAIT : IDLE);
                    CHECK(pend.size() == c.pending(), "pending mirror");
                    if (!n)
                        CHECK(!frame_now && !full_now && !idle_now, "held back: pending %u, need %u, quiet %u",
                              (unsigned)pend.size(), need, quiet);
                    if (n) {
                        CHECK(n >= 1 && n <= cap, "packet size %u (cap %u)", n, cap);
                        CHECK(frame_now ? n == need : ((full_now || idle_now) && n == pend.size()),
                              "flush rule: n %u, pending %u, need %u, quiet %u", n, (unsigned)pend.size(), need, quiet);
                        c.take(out, n);
                        outs.insert(outs.end(), out, out + n);
                        packets++;
                    }
                }
                t = t_last + gap;
                if (!c.push(piece[i], t)) {
                    uint16_t n = c.due(t);
                    CHECK(n == cap, "refused push only when full (%u / %u)", n, cap);
                    c.take(out, n);
                    outs.insert(outs.end(), out, out + n);
                    CHECK(c.push(piece[i], t), "push after take");
                }
                t_last = t;
                in.push_back(piece[i]);
            }
        }
        /* drain */
        uint16_t n;
        while ((n = c.due(t + WAIT + 1)) > 0) {
            c.take(out, n);
            outs.insert(outs.end(), out, out + n);
        }
        CHECK(c.pending() == 0, "drained");
        CHECK(outs == in, "run %d: emitted packets concatenate to the input (%u vs %u bytes, %d packets)", run,
              (unsigned)outs.size(), (unsigned)in.size(), packets);
    }
}

static void test_ring(void)
{
    ByteRing r;
    uint8_t buf[300], out[2048];
    for (int i = 0; i < 300; i++)
        buf[i] = (uint8_t)i;
    CHECK(r.space() == ByteRing::SIZE - 1u && r.used() == 0, "empty ring");
    CHECK(r.put(buf, 250) && r.put(buf, 250) && r.put(buf, 250) && r.put(buf, 250), "four 250-byte packets fit");
    CHECK(!r.put(buf, 250) && r.dropped_packets() == 1 && r.dropped_bytes() == 250, "the fifth is dropped whole");
    CHECK(r.used() == 1000, "1000 used");
    uint16_t n = r.get(out, 2048);
    CHECK(n == 1000, "get all");
    bool ok = true;
    for (int i = 0; i < 1000; i++)
        ok = ok && out[i] == (uint8_t)(i % 250);
    CHECK(ok, "ring keeps order");
    /* wrap many times with random sizes */
    std::vector<uint8_t> in, got;
    for (int k = 0; k < 5000; k++) {
        uint16_t m = (uint16_t)(1 + rnd() % 250);
        for (int i = 0; i < m; i++)
            buf[i] = (uint8_t)rnd();
        if (r.put(buf, m))
            in.insert(in.end(), buf, buf + m);
        if (rnd() % 3) {
            n = r.get(out, (uint16_t)(rnd() % 600));
            got.insert(got.end(), out, out + n);
        }
    }
    n = r.get(out, 2048);
    got.insert(got.end(), out, out + n);
    CHECK(got == in, "random wraps: order and content kept (%u / %u)", (unsigned)got.size(), (unsigned)in.size());
    /* review LB-2: the exactly-full boundary, at several ring offsets */
    for (int off = 0; off < 5; off++) {
        ByteRing q;
        uint16_t i;
        bool fill = true;
        for (i = 0; i < (uint16_t)(off * 211); i++) {             /* move head / tail to another offset */
            uint8_t b = 0;
            q.put(&b, 1);
            q.get(&b, 1);
        }
        for (i = 0; i < ByteRing::SIZE - 1u; i++) {
            const uint8_t b = (uint8_t)(i * 7u + (unsigned)off);
            fill = fill && q.put(&b, 1);
        }
        CHECK(fill && q.used() == ByteRing::SIZE - 1u && q.space() == 0, "offset %d: 1023 single bytes fit, used 1023", off);
        uint8_t one = 0xEE;
        CHECK(!q.put(&one, 1) && q.dropped_packets() == 1 && q.used() == ByteRing::SIZE - 1u,
              "offset %d: the 1024th byte refused, nothing lost", off);
        n = q.get(out, 2048);
        bool same = n == ByteRing::SIZE - 1u;
        for (i = 0; same && i < n; i++)
            same = out[i] == (uint8_t)(i * 7u + (unsigned)off);
        CHECK(same, "offset %d: the full ring reads back unchanged (%u)", off, n);
        /* space() == k: put(k) fits, put(k + 1) does not */
        for (int rep = 0; rep < 20; rep++) {
            ByteRing w;
            const uint16_t pre = (uint16_t)(ByteRing::SIZE - 1u - rnd() % 299u);   /* space 0..298 (<= sizeof buf) */
            for (i = 0; i < pre && w.space() > 0; i++) {
                uint8_t b = (uint8_t)i;
                w.put(&b, 1);
            }
            const uint16_t k = w.space();
            CHECK(k + 1u <= sizeof buf ? !w.put(buf, (uint16_t)(k + 1u)) : true, "space %u: put(k + 1) refused", k);
            CHECK(k <= sizeof buf ? w.put(buf, k) && w.space() == 0 : true, "space %u: put(k) fits", k);
        }
    }
}

static void test_mac(void)
{
    uint8_t m[6];
    CHECK(bridge_parse_mac("02:CA:5a:00:00:01", m) && m[0] == 0x02 && m[1] == 0xCA && m[2] == 0x5A && m[5] == 0x01, "mac ok");
    CHECK(!bridge_parse_mac("02:CA:5A:00:00", m), "5 bytes");
    CHECK(!bridge_parse_mac("02:CA:5A:00:00:01:", m), "trailing colon");
    CHECK(!bridge_parse_mac("02-CA-5A-00-00-01", m), "dashes");
    CHECK(!bridge_parse_mac("02:CA:5G:00:00:01", m), "bad hex");
    CHECK(!bridge_parse_mac("", m), "empty");
}

int main(void)
{
    test_basic();
    test_random();
    test_batch();
    test_ring();
    test_mac();
    printf("BRIDGE_CORE %d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
