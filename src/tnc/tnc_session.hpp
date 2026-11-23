#pragma once

#include "modem_adapter.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ultra::tnc {

class TNCSession {
public:
    using EmitFn = std::function<void(std::string_view line)>;
    using DataOutFn = std::function<void(const std::vector<uint8_t>&)>;

    TNCSession(ModemAdapter& modem, EmitFn cmd_emit, DataOutFn data_out);

    bool handleControlLine(std::string_view line);
    void handleDataBytes(const std::vector<uint8_t>& bytes);

    void onModemConnected(const std::string& src, const std::string& dst, int bw);
    void onModemDisconnected();
    void onModemPTT(bool on);
    void onModemDataReceived(const std::vector<uint8_t>& bytes);
    void onModemBufferLevel(int bytes);
    void onModemSNR(float db);
    void onModemBitrate(int bps);
    void onModemIncomingCall(const std::string& peer);

    void tick(uint32_t elapsed_ms);

    State getState() const { return state_; }
    const std::string& getMyCall() const { return mycall_; }

    // Pure encoder for the on-wire payload framing. Picks compression
    // when it actually saves bytes after the 1-byte marker overhead
    // and falls back to raw otherwise. Static and side-effect free so
    // tests can probe each branch (raw, compressible, uncompressible,
    // below-threshold, compression-disabled) without spinning up a
    // full session. Returns wire bytes ready for ModemAdapter::sendBinary.
    static std::vector<uint8_t> encodePayloadForWire(
        const std::vector<uint8_t>& payload, bool compression_enabled);

private:
    ModemAdapter& modem_;
    EmitFn cmd_emit_;
    DataOutFn data_out_;

    State state_ = State::IDLE;
    std::string mycall_;
    std::vector<std::string> secondary_calls_;
    int bandwidth_hz_ = 2300;
    bool compression_enabled_ = false;
    bool chat_enabled_ = false;
    bool cwid_enabled_ = false;
    bool pending_inbound_ = false;
    // The host's standing LISTEN ON (until LISTEN OFF). A session that ends returns to listening:
    // it used to return to READY, so the second call the modem accepted never reached the host
    // (BUG-TNC-LISTEN-LOST-AFTER-SESSION).
    bool listen_enabled_ = false;
    uint32_t iamalive_timer_ms_ = 0;
    int last_buffer_level_ = -1;
    uint32_t last_buffer_emit_ms_ = 0;
    int pending_buffer_level_ = -1;
    float last_snr_db_ = 0.0f;
    int last_bitrate_bps_ = 0;

    // TCP→modem batching: TCP delivers user data in many small chunks,
    // but Connection::sendPayload() replaces (not appends to) its pending
    // fragment queue on every call. Calling sendBinary() per TCP chunk
    // therefore strands fragments mid-transfer. Accumulate bytes in a
    // local buffer and flush via one sendBinary() call after a brief
    // quiet period — same shape cli_simulator uses, which Connection
    // handles correctly.
    std::vector<uint8_t> data_tx_buffer_;
    uint32_t data_tx_quiet_ms_ = 0;
    static constexpr uint32_t kDataTxFlushQuietMs = 200;
    static constexpr size_t kDataTxFlushSizeBytes = 64 * 1024;
    // Traffic-class boundary (PHY_ADAPTATION_DESIGN §3/§7). A flushed block at or below
    // this size is treated as INTERACTIVE (Winlink-B2F control exchange — banners,
    // proposals, FS answers — and short messages) and sent on the NON-BURST short-LDPC
    // (z=27) SelectiveRepeatARQ path with the ISS/IRS turn gate: low latency, survives by
    // brevity. A larger block is a BULK transfer and pays for the burst + long-LDPC (z=81)
    // + deep-interleave file path. The burst path's group/anchor machinery is wrong for
    // tiny alternating messages (it can't re-acquire burst timing on every turn-flip).
    static constexpr size_t kInteractiveMaxBytes = 4096;
    // The accumulated buffer is flushed as ONE modem file-transfer unit (Z=81 burst path),
    // staged to a temp file. Counter keeps each flush's temp name unique; last path is
    // cleaned up on the next flush / disconnect (its transfer has progressed by then).
    uint64_t tx_file_counter_ = 0;
    std::string last_tx_temp_path_;

    // ACCUMULATION (default on; opt out with ULTRA_TNC_ACCUM_DISABLE for the legacy
    // per-chunk flush). The VARA data port delivers the host's byte stream in many small
    // TCP writes; rather than shipping each as its own short frame, we accumulate and
    // flush larger units — a burst-worth (kBurstFlushTargetBytes) when the TX path is
    // free, or whatever remains once the host goes idle. We report HONEST BUFFER
    // throughout (the host's own VARA flow control then paces how far ahead it runs), so
    // this is protocol-AGNOSTIC: it works for any VARA-HF host, not just Winlink/PAT.
    bool bulk_accum_ = false;
    // SIZE TARGET for the burst-flush: once the staging buffer reaches this AND the modem
    // TX path is free, ship it as one burst transport unit instead of waiting for the
    // idle gap. Tunable; ~4 KB ≈ a few burst groups.
    static constexpr size_t kBurstFlushTargetBytes = 4096;
    // ORDERING INVARIANT: a single continuous host send must traverse ONE transport. The
    // burst path (sendFile) and interactive path (sendBinary) are independent ARQ
    // mechanisms with no shared sequence space, so striping one byte stream across both
    // reassembles out of order → corruption. Once any chunk of the current feed has
    // bursted, force ALL remaining chunks onto the burst path. Reset per connection.
    bool bulk_burst_started_ = false;
    // FLUSH PROBES (diagnostics): ULTRA_TNC_ACCUM_PROBE logs each flush size + route then
    // fast-exits at the first burst (isolates the accumulation layer, no air TX);
    // ULTRA_TNC_FLUSH_LOG just logs every flush size + route in the live path.
    bool accum_probe_ = false;
    bool flush_log_ = false;
    // Idle gap meaning "the host has paused" — flush whatever remains (the sub-target
    // tail of a send, or a short message).
    static constexpr uint32_t kDataTxBulkQuietMs = 1500;
    // A real VARA modem reports buffer state continuously. onModemBufferLevel emits on
    // enqueue/ACK events, but if the engine backlog sits unchanged for a long stretch
    // (e.g. a slow burst between group ACKs) no event fires; re-emit the current honest
    // level this often so a host's Flush() inactivity timer (VARA's is ~60 s) never
    // starves. 3x margin.
    static constexpr uint32_t kBufferKeepaliveMs = 20000;

    static std::pair<std::string, std::string> parseCommand(std::string_view line);

    void endSessionForHost();

    void emitOK();
    void emitWrong();
    void emitVersion();
    void emitConnected(const std::string& src, const std::string& dst, int bw);
    void emitDisconnected();
    void emitPtt(bool on);
    void emitBuffer(int bytes);
    void emitSN(float db);
    void emitBitrate(int bps);
    void emitIamalive();
    void emitPending();
    void emitCancelPending();

    void cmdMyCall(std::string_view args);
    void cmdBandwidth(int hz);
    void cmdListen(std::string_view args);
    void cmdConnect(std::string_view args);
    void cmdDisconnect();
    void cmdAbort();
    void cmdCompression(std::string_view args);
    void cmdChat(std::string_view args);
    void cmdCwid(std::string_view args);
    void cmdVersion(std::string_view args);
    void cmdBuffer(std::string_view args);
    void cmdSn(std::string_view args);
    void cmdBitrate(std::string_view args);
    void cmdPublic(std::string_view args);
    void cmdP2P(std::string_view args);
    void cmdWinlink(std::string_view args);
    void cmdIgnoreKissDcd(std::string_view args);
    void cmdIntegerNoop(std::string_view args);
    void cmdStats(std::string_view args);

    void flushDataTxBuffer();
};

} // namespace ultra::tnc
