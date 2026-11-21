#include "tnc/tnc_session.hpp"

#include "protocol/compression.hpp"
#include "ultra/version.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <system_error>
#include <string>
#include <string_view>

namespace ultra::tnc {
namespace {

constexpr uint32_t kIAmAliveIntervalMs = 60000;
constexpr uint32_t kBufferEmitIntervalMs = 1000;

// Single-byte wire-format header on the modem-side payload to signal
// compression. Stripped on RX, transparent to the data-port client.
// Both peers must run a compression-aware ultra_tnc; pre-compression
// builds will see the marker as part of the data (incompatible).
constexpr uint8_t kPayloadMarkerRaw = 0x00;
constexpr uint8_t kPayloadMarkerDeflate = 0x01;

std::string toUpper(std::string_view value) {
    std::string out;
    out.reserve(value.size());
    for (char ch : value) {
        out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
    }
    return out;
}

bool isSpace(char ch) {
    return std::isspace(static_cast<unsigned char>(ch)) != 0;
}

std::string_view trimView(std::string_view value) {
    while (!value.empty() && isSpace(value.front())) {
        value.remove_prefix(1);
    }
    while (!value.empty() && isSpace(value.back())) {
        value.remove_suffix(1);
    }
    return value;
}

std::vector<std::string> splitWhitespace(std::string_view value) {
    std::vector<std::string> tokens;
    value = trimView(value);
    while (!value.empty()) {
        size_t end = 0;
        while (end < value.size() && !isSpace(value[end])) {
            ++end;
        }
        tokens.emplace_back(value.substr(0, end));
        value.remove_prefix(end);
        value = trimView(value);
    }
    return tokens;
}

bool parseNonNegativeInt(std::string_view value, int& out) {
    value = trimView(value);
    if (value.empty()) {
        return false;
    }

    int parsed = 0;
    const char* first = value.data();
    const char* last = value.data() + value.size();
    auto [ptr, ec] = std::from_chars(first, last, parsed);
    if (ec != std::errc{} || ptr != last || parsed < 0) {
        return false;
    }

    out = parsed;
    return true;
}

bool isValidCallsignToken(std::string_view raw) {
    if (raw.empty() || raw.size() > 15) {
        return false;
    }

    const size_t dash = raw.find('-');
    if (dash != std::string_view::npos && raw.find('-', dash + 1) != std::string_view::npos) {
        return false;
    }

    const std::string_view base = dash == std::string_view::npos ? raw : raw.substr(0, dash);
    if (base.size() < 3 || base.size() > 10) {
        return false;
    }
    for (char ch : base) {
        if (std::isalnum(static_cast<unsigned char>(ch)) == 0) {
            return false;
        }
    }

    if (dash == std::string_view::npos) {
        return true;
    }

    const std::string_view ssid = raw.substr(dash + 1);
    if (ssid.empty() || ssid.size() > 2) {
        return false;
    }
    for (char ch : ssid) {
        if (std::isdigit(static_cast<unsigned char>(ch)) == 0) {
            return false;
        }
    }

    int ssid_value = 0;
    if (!parseNonNegativeInt(ssid, ssid_value)) {
        return false;
    }
    return ssid_value <= 15;
}

std::vector<std::string> parseCallsignList(std::string_view args) {
    auto tokens = splitWhitespace(args);
    for (std::string& token : tokens) {
        token = toUpper(token);
    }
    return tokens;
}

bool isReadyAfterDisconnect(const std::string& mycall) {
    return !mycall.empty();
}

std::string formatFloatOneDecimal(float value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.1f", value);
    return std::string(buffer);
}

} // namespace

const char* stateToString(State state) {
    switch (state) {
    case State::IDLE:
        return "IDLE";
    case State::READY:
        return "READY";
    case State::LISTENING:
        return "LISTENING";
    case State::CONNECTING:
        return "CONNECTING";
    case State::CONNECTED:
        return "CONNECTED";
    case State::DISCONNECTING:
        return "DISCONNECTING";
    }
    return "UNKNOWN";
}

TNCSession::TNCSession(ModemAdapter& modem, EmitFn cmd_emit, DataOutFn data_out)
    : modem_(modem),
      cmd_emit_(std::move(cmd_emit)),
      data_out_(std::move(data_out)) {
    // Flush-batching is the DEFAULT: accumulate the host's data-port stream and flush at a
    // burst-worth (kBurstFlushTargetBytes) or when the host goes idle, so a multi-frame body
    // ships as fuller bursts instead of one tiny burst per host chunk. The BUFFER level is
    // ALWAYS reported HONESTLY (handleDataBytes -> onModemBufferLevel = true backlog); the old
    // under-report-to-milk-PAT trick is GONE. Opt out with ULTRA_TNC_ACCUM_DISABLE for the
    // legacy per-chunk 200 ms flush (each host chunk bursts on its own via the unified path).
    bulk_accum_ = std::getenv("ULTRA_TNC_ACCUM_DISABLE") == nullptr;
    accum_probe_ = std::getenv("ULTRA_TNC_ACCUM_PROBE") != nullptr;
    flush_log_ = std::getenv("ULTRA_TNC_FLUSH_LOG") != nullptr;
    if (accum_probe_) {
        bulk_accum_ = true;  // measuring the bulk-accumulate body size requires bulk mode
    }
}

bool TNCSession::handleControlLine(std::string_view line) {
    const bool has_cr_or_lf = line.find_first_of("\r\n") != std::string_view::npos;
    const std::string_view trimmed = trimView(line);

    if (trimmed.empty()) {
        if (has_cr_or_lf) {
            emitWrong();
        }
        return false;
    }

    if (has_cr_or_lf) {
        emitWrong();
        return false;
    }

    auto [command, args] = parseCommand(trimmed);
    if (command.empty()) {
        return false;
    }

    if (command == "MYCALL") {
        cmdMyCall(args);
    } else if (command == "LISTEN") {
        cmdListen(args);
    } else if (command == "CONNECT") {
        cmdConnect(args);
    } else if (command == "DISCONNECT") {
        if (!trimView(args).empty()) {
            emitWrong();
        } else {
            cmdDisconnect();
        }
    } else if (command == "ABORT") {
        if (!trimView(args).empty()) {
            emitWrong();
        } else {
            cmdAbort();
        }
    } else if (command == "COMPRESSION") {
        cmdCompression(args);
    } else if (command == "CHAT") {
        cmdChat(args);
    } else if (command == "CWID") {
        cmdCwid(args);
    } else if (command == "VERSION") {
        cmdVersion(args);
    } else if (command == "BUFFER") {
        cmdBuffer(args);
    } else if (command == "SN") {
        cmdSn(args);
    } else if (command == "BITRATE") {
        cmdBitrate(args);
    } else if (command == "PUBLIC") {
        cmdPublic(args);
    } else if (command == "P2P") {
        cmdP2P(args);
    } else if (command == "WINLINK") {
        cmdWinlink(args);
    } else if (command == "IGNOREKISSDCD") {
        cmdIgnoreKissDcd(args);
    } else if (command == "RETRIES" || command == "CALLINT") {
        cmdIntegerNoop(args);
    } else if (command == "STATS") {
        cmdStats(args);
    } else if (command.rfind("BW", 0) == 0) {
        if (!trimView(args).empty()) {
            emitWrong();
            return true;
        }

        int hz = 0;
        const std::string_view value(command.data() + 2, command.size() - 2);
        if (!parseNonNegativeInt(value, hz) || (hz != 500 && hz != 2300 && hz != 2750)) {
            emitWrong();
            return true;
        }
        cmdBandwidth(hz);
    } else {
        emitWrong();
    }

    return true;
}

void TNCSession::handleDataBytes(const std::vector<uint8_t>& bytes) {
    if (state_ != State::CONNECTED || bytes.empty()) {
        return;
    }
    data_tx_buffer_.insert(data_tx_buffer_.end(), bytes.begin(), bytes.end());
    data_tx_quiet_ms_ = 0;
    const bool modem_ready = modem_.getTxBackloggBytes() == 0;
    if (data_tx_buffer_.size() >= kDataTxFlushSizeBytes) {
        flushDataTxBuffer();  // hard ceiling — bound the staging buffer
    } else if (bulk_accum_ && modem_ready &&
               data_tx_buffer_.size() >= kBurstFlushTargetBytes) {
        // Accumulated a burst-worth and the TX path is free: ship it now as one burst
        // transport unit rather than waiting for the idle timer. The next chunk
        // accumulates while this one transmits.
        flushDataTxBuffer();
    }
    // HONEST BUFFER (VARA-HF spec: "BUFFER <bytes> ... Sent when VARA adds data to
    // queue"). Report the TRUE transmit-queue depth — bytes the host has handed us that
    // are not yet ACKed away (our staging buffer + the engine's unacked backlog). ANY
    // conformant host paces itself on this; its own flow control bounds how far ahead it
    // runs, so we never under-report to coax more data out of it (which would both lie
    // about queue depth and be a host-specific hack).
    onModemBufferLevel(modem_.getTxBackloggBytes());
}

void TNCSession::onModemConnected(const std::string& src, const std::string& dst, int bw) {
    bulk_burst_started_ = false;  // fresh transport decision per connection
    if (state_ == State::LISTENING) {
        if (!pending_inbound_) {
            emitPending();
        }
        pending_inbound_ = false;
        state_ = State::CONNECTED;
        bandwidth_hz_ = bw;
        emitConnected(src, dst, bw);
        return;
    }

    if (state_ == State::CONNECTING) {
        pending_inbound_ = false;
        state_ = State::CONNECTED;
        bandwidth_hz_ = bw;
        emitConnected(src, dst, bw);
    }
}

void TNCSession::onModemDisconnected() {
    if (pending_inbound_ && state_ == State::LISTENING) {
        pending_inbound_ = false;
        emitCancelPending();
        return;
    }

    if (state_ == State::CONNECTED || state_ == State::CONNECTING || state_ == State::DISCONNECTING) {
        pending_inbound_ = false;
        state_ = isReadyAfterDisconnect(mycall_) ? State::READY : State::IDLE;
        data_tx_buffer_.clear();
        if (!last_tx_temp_path_.empty()) {
            std::error_code ec;
            std::filesystem::remove(last_tx_temp_path_, ec);
            last_tx_temp_path_.clear();
        }
        data_tx_quiet_ms_ = 0;
        emitDisconnected();
    }
}

void TNCSession::onModemPTT(bool on) {
    emitPtt(on);
}

void TNCSession::onModemDataReceived(const std::vector<uint8_t>& bytes) {
    if (state_ != State::CONNECTED || bytes.empty() || !data_out_) {
        return;
    }

    const uint8_t marker = bytes.front();
    if (marker == kPayloadMarkerDeflate) {
        std::vector<uint8_t> compressed(bytes.begin() + 1, bytes.end());
        if (auto decompressed = ultra::protocol::Compression::decompress(compressed)) {
            data_out_(*decompressed);
        }
        // Decompression failure is silently dropped — the peer claimed
        // deflate but produced bytes we can't decode. Don't forward
        // garbage to the data-port client.
        return;
    }
    if (marker == kPayloadMarkerRaw) {
        std::vector<uint8_t> payload(bytes.begin() + 1, bytes.end());
        data_out_(payload);
        return;
    }

    // Unknown marker — most likely a peer running a pre-compression build
    // of ultra_tnc that doesn't prepend a header. Pass the bytes through
    // unchanged so old/new combinations still mostly work, accepting that
    // the first byte will look anomalous to the data-port client.
    data_out_(bytes);
}

void TNCSession::onModemBufferLevel(int bytes) {
    if (bytes < 0) {
        bytes = 0;
    }
    // HONEST queue depth (VARA-HF spec): the engine's unacked backlog
    // (getTxBacklogBytes already excludes ACKed ARQ slots, so it decrements on ACK as
    // the spec's "VARA removes acked bytes from queue" requires) PLUS our not-yet-
    // shipped staging buffer. data_tx_buffer_ holds bytes the host wrote that we have
    // not handed to the engine yet — they're in the queue, so an honest modem counts
    // them. The host's Flush() trusts BUFFER 0 to mean fully delivered; including the
    // staging bytes keeps us from signalling 0 while data is still unsent.
    bytes += static_cast<int>(data_tx_buffer_.size());

    if (bytes == last_buffer_level_ && pending_buffer_level_ < 0) {
        return;
    }

    // Pat's Flush() blocks on BUFFER 0; any delay there directly
    // delays the application releasing the connection. Zero-going
    // transitions skip the rate limit so Flush() unblocks ASAP.
    // Non-zero updates stay rate-limited so a long send doesn't
    // flood the cmd port with intermediate BUFFER N values.
    const bool first_emit = (last_buffer_level_ < 0);
    const bool cooldown_elapsed = (last_buffer_emit_ms_ >= kBufferEmitIntervalMs);
    if (first_emit || cooldown_elapsed || bytes == 0) {
        pending_buffer_level_ = -1;
        emitBuffer(bytes);
        return;
    }

    pending_buffer_level_ = bytes;
}

void TNCSession::onModemSNR(float db) {
    last_snr_db_ = db;
    if (chat_enabled_ && state_ == State::CONNECTED) {
        emitSN(db);
    }
}

void TNCSession::onModemBitrate(int bps) {
    last_bitrate_bps_ = bps;
    emitBitrate(bps);
}

void TNCSession::onModemIncomingCall(const std::string& peer) {
    (void)peer;
    if (state_ == State::LISTENING && !pending_inbound_) {
        pending_inbound_ = true;
        emitPending();
    }
}

void TNCSession::tick(uint32_t elapsed_ms) {
    iamalive_timer_ms_ += elapsed_ms;
    while (iamalive_timer_ms_ >= kIAmAliveIntervalMs) {
        iamalive_timer_ms_ -= kIAmAliveIntervalMs;
        emitIamalive();
    }

    if (last_buffer_emit_ms_ <= std::numeric_limits<uint32_t>::max() - elapsed_ms) {
        last_buffer_emit_ms_ += elapsed_ms;
    } else {
        last_buffer_emit_ms_ = std::numeric_limits<uint32_t>::max();
    }

    if (pending_buffer_level_ >= 0 && last_buffer_emit_ms_ >= kBufferEmitIntervalMs &&
        pending_buffer_level_ != last_buffer_level_) {
        const int bytes = pending_buffer_level_;
        pending_buffer_level_ = -1;
        emitBuffer(bytes);
    }

    // BUFFER keepalive (bulk mode): while a transfer is pending (last reported
    // level > 0) but the backlog has been frozen long enough that no fresh
    // BUFFER update has gone out, re-emit the current level so Pat's Flush()
    // 60 s timer keeps resetting through burst-path group-ACK stalls. Without
    // this the burst delivers correctly but Pat aborts the exchange mid-send.
    if (bulk_accum_ && state_ == State::CONNECTED && last_buffer_level_ > 0 &&
        last_buffer_emit_ms_ >= kBufferKeepaliveMs) {
        emitBuffer(last_buffer_level_);
    }

    if (state_ == State::CONNECTED && !data_tx_buffer_.empty()) {
        data_tx_quiet_ms_ += elapsed_ms;
        // Flush only when the modem TX path is free — a sendFile while the previous
        // burst is still transmitting fails ("file transfer in progress") and would
        // retry every tick, churning. Two triggers fire it in bulk mode: a burst-worth
        // is ready (SIZE TARGET), or PAT has gone idle (kDataTxBulkQuietMs) meaning the
        // body/message is complete — flush whatever remains (the sub-target tail, or a
        // small interactive message). Legacy (bulk off) keeps the per-chunk 200 ms flush.
        const bool modem_ready = !bulk_accum_ || modem_.getTxBackloggBytes() == 0;
        const uint32_t idle_thresh = bulk_accum_ ? kDataTxBulkQuietMs : kDataTxFlushQuietMs;
        const bool have_burst_worth =
            bulk_accum_ && data_tx_buffer_.size() >= kBurstFlushTargetBytes;
        const bool idle_done = data_tx_quiet_ms_ >= idle_thresh;
        if (modem_ready && (have_burst_worth || idle_done)) {
            flushDataTxBuffer();
        }
    }
}

std::vector<uint8_t> TNCSession::encodePayloadForWire(
    const std::vector<uint8_t>& payload, bool compression_enabled) {
    std::vector<uint8_t> wire;
    wire.reserve(payload.size() + 1);

    bool sent_compressed = false;
    if (compression_enabled &&
        payload.size() >= ultra::protocol::Compression::MIN_COMPRESS_SIZE) {
        if (auto compressed = ultra::protocol::Compression::compress(payload)) {
            // Only ship the compressed copy if it actually pays for the
            // 1-byte marker overhead. On already-compressed or random
            // input, deflate often expands; fall back to raw.
            if (compressed->size() + 1 < payload.size()) {
                wire.push_back(kPayloadMarkerDeflate);
                wire.insert(wire.end(), compressed->begin(), compressed->end());
                sent_compressed = true;
            }
        }
    }
    if (!sent_compressed) {
        wire.push_back(kPayloadMarkerRaw);
        wire.insert(wire.end(), payload.begin(), payload.end());
    }
    return wire;
}

void TNCSession::flushDataTxBuffer() {
    if (data_tx_buffer_.empty()) {
        data_tx_quiet_ms_ = 0;
        return;
    }
    auto wire = encodePayloadForWire(data_tx_buffer_, compression_enabled_);

    // TRAFFIC-CLASS ROUTING (PHY_ADAPTATION_DESIGN §3/§7). Small interactive blocks — the
    // Winlink-B2F control exchange and short messages — go on the NON-BURST short-LDPC
    // (z=27) SelectiveRepeatARQ path via sendBinary(). That path uses the ISS/IRS turn gate
    // (queued_payloads_ / TURN_REQUEST / TURNOVER) so the two stations alternate cleanly,
    // and per-frame sync handles each turn-flip — unlike the burst file path, whose group
    // anchor can't re-acquire timing when the link flips for every tiny B2F message. The RX
    // side already delivers both transports to the data port (TNCBridge wires BOTH
    // setDataReceivedCallback and setFileReceivedCallback to postModemDataReceived).
    // Route to burst when this flush is a burst-worth (> kInteractiveMaxBytes) OR the
    // current feed already bursted (ORDERING INVARIANT: keep the whole body on one
    // transport — a sub-target body tail must NOT drop to the interactive sendBinary
    // path, or it reassembles out of order against the bursted prefix → corruption).
    const bool route_burst =
        wire.size() > kInteractiveMaxBytes || (bulk_accum_ && bulk_burst_started_);

    if (accum_probe_ || flush_log_) {
        // STEP-1: report what PAT handed us at this flush. Interactive (handshake)
        // flushes still go to the modem so the B2F proposal reaches the responder
        // and PAT releases the body; under accum_probe_ the BODY's burst-flush is
        // short-circuited (fast-exit). Under flush_log_ alone, nothing is forced or
        // skipped — it just observes every flush size + route in the live path.
        static size_t probe_flush_n = 0;
        std::fprintf(stderr,
            "[ACCUM-PROBE] flush #%zu: accumulated raw=%zu B, wire(after-compress)=%zu B, "
            "route=%s\n",
            ++probe_flush_n, data_tx_buffer_.size(), wire.size(),
            route_burst ? "BURST" : "interactive");
        std::fflush(stderr);
        if (accum_probe_ && route_burst) {
            std::fprintf(stderr,
                "[ACCUM-PROBE] ===== WOULD BURST: %zu wire bytes (%zu raw body bytes) as ONE "
                "z=81 file transfer. Fast-exit before air TX. =====\n",
                wire.size(), data_tx_buffer_.size());
            std::fflush(stderr);
            std::_Exit(0);
        }
    }

    if (!route_burst) {
        if (modem_.sendBinary(wire)) {
            data_tx_buffer_.clear();
        }
        // else: engine refused (queue full / not CONNECTED) — keep data_tx_buffer_ for the
        // next quiet-period retry (Pat sees BUFFER N stay nonzero and backs off).
        data_tx_quiet_ms_ = 0;
        return;
    }

    // BULK (> kInteractiveMaxBytes): ship as ONE modem FILE TRANSFER (Z=81 burst-file path),
    // matching what the GUI does (file_transfer_ SENDING -> selectBurstLiftingZ()==81). Stage
    // `wire` to a temp file (FileTransferController reads it whole at startSend) and hand the
    // far side back the same wire bytes, which TNCSession::onModemDataReceived decodes +
    // delivers out its data port. The callsign keeps the name unique across the two ultra_tnc
    // processes that may share /tmp (e.g. the OTASim test rig).
    std::error_code ec;
    if (!last_tx_temp_path_.empty()) {
        std::filesystem::remove(last_tx_temp_path_, ec);
        last_tx_temp_path_.clear();
    }
    const std::string call = mycall_.empty() ? "tnc" : mycall_;
    const std::string temp_path =
        (std::filesystem::temp_directory_path() /
         ("ultra_tnc_tx_" + call + "_" + std::to_string(tx_file_counter_++) + ".bin"))
            .string();
    {
        std::ofstream out(temp_path, std::ios::binary | std::ios::trunc);
        if (out.good()) {
            out.write(reinterpret_cast<const char*>(wire.data()),
                      static_cast<std::streamsize>(wire.size()));
        }
    }
    if (modem_.sendFile(temp_path)) {
        data_tx_buffer_.clear();
        last_tx_temp_path_ = temp_path;
        // Lock the rest of this feed onto the burst transport (ordering invariant).
        if (bulk_accum_) bulk_burst_started_ = true;
    } else {
        // Engine refused (queue full / not CONNECTED): keep data_tx_buffer_ intact so the
        // next quiet-period flush retries. Pat sees BUFFER N stay nonzero and backs off.
        std::filesystem::remove(temp_path, ec);
    }
    data_tx_quiet_ms_ = 0;
}

std::pair<std::string, std::string> TNCSession::parseCommand(std::string_view line) {
    line = trimView(line);
    size_t split = 0;
    while (split < line.size() && !isSpace(line[split])) {
        ++split;
    }

    std::string command = toUpper(line.substr(0, split));
    std::string args;
    if (split < line.size()) {
        args = std::string(trimView(line.substr(split)));
    }

    return {std::move(command), std::move(args)};
}

void TNCSession::emitOK() {
    if (cmd_emit_) {
        cmd_emit_("OK\r");
    }
}

void TNCSession::emitWrong() {
    if (cmd_emit_) {
        cmd_emit_("WRONG\r");
    }
}

void TNCSession::emitVersion() {
    if (cmd_emit_) {
        // pat-vara's pubsub dispatches incoming lines by prefix, and its
        // Version() subscribes to lines starting with "VERSION" or "WRONG".
        // The legacy Mercury-style "VARA version ... registered" banner
        // matches neither, so Pat hangs (or logs "got a vara command I
        // wasn't expecting"). Lead with "VERSION " so pat-vara's
        // strings.TrimPrefix(str, "VERSION ") yields the version.
        cmd_emit_(std::string("VERSION ") + kProjectUltraVersion + "\r");
    }
}

void TNCSession::emitConnected(const std::string& src, const std::string& dst, int bw) {
    if (cmd_emit_) {
        const std::string line = "CONNECTED " + src + " " + dst + " " + std::to_string(bw) + "\r";
        cmd_emit_(line);
    }
}

void TNCSession::emitDisconnected() {
    if (cmd_emit_) {
        cmd_emit_("DISCONNECTED\r");
    }
}

void TNCSession::emitPtt(bool on) {
    if (cmd_emit_) {
        cmd_emit_(on ? "PTT ON\r" : "PTT OFF\r");
    }
}

void TNCSession::emitBuffer(int bytes) {
    if (bytes < 0) {
        bytes = 0;
    }
    last_buffer_level_ = bytes;
    last_buffer_emit_ms_ = 0;

    if (cmd_emit_) {
        const std::string line = "BUFFER " + std::to_string(bytes) + "\r";
        cmd_emit_(line);
    }
}

void TNCSession::emitSN(float db) {
    if (cmd_emit_) {
        const std::string line = "SN " + formatFloatOneDecimal(db) + "\r";
        cmd_emit_(line);
    }
}

void TNCSession::emitBitrate(int bps) {
    if (bps < 0) {
        bps = 0;
    }
    if (cmd_emit_) {
        const std::string line = "BITRATE (0) " + std::to_string(bps) + " BPS\r";
        cmd_emit_(line);
    }
}

void TNCSession::emitIamalive() {
    if (cmd_emit_) {
        cmd_emit_("IAMALIVE\r");
    }
}

void TNCSession::emitPending() {
    if (cmd_emit_) {
        cmd_emit_("PENDING\r");
    }
}

void TNCSession::emitCancelPending() {
    if (cmd_emit_) {
        cmd_emit_("CANCELPENDING\r");
    }
}

void TNCSession::cmdMyCall(std::string_view args) {
    if (state_ == State::CONNECTING || state_ == State::CONNECTED || state_ == State::DISCONNECTING) {
        emitWrong();
        return;
    }

    auto calls = parseCallsignList(args);
    if (calls.empty() || calls.size() > 5) {
        emitWrong();
        return;
    }

    for (const std::string& call : calls) {
        if (!isValidCallsignToken(call)) {
            emitWrong();
            return;
        }
    }

    mycall_ = calls.front();
    secondary_calls_.assign(calls.begin() + 1, calls.end());
    pending_inbound_ = false;
    state_ = State::READY;
    modem_.setMyCall(calls);
    emitOK();
}

void TNCSession::cmdBandwidth(int hz) {
    if (state_ == State::CONNECTING || state_ == State::CONNECTED || state_ == State::DISCONNECTING) {
        emitWrong();
        return;
    }

    bandwidth_hz_ = hz;
    modem_.setBandwidth(hz);
    emitOK();
}

void TNCSession::cmdListen(std::string_view args) {
    const auto tokens = splitWhitespace(args);
    if (tokens.size() != 1 || mycall_.empty()) {
        emitWrong();
        return;
    }

    const std::string mode = toUpper(tokens.front());
    if (mode != "ON" && mode != "OFF" && mode != "CQ") {
        emitWrong();
        return;
    }

    if (state_ == State::CONNECTED || state_ == State::CONNECTING || state_ == State::DISCONNECTING) {
        emitWrong();
        return;
    }

    if (mode == "ON" || mode == "CQ") {
        if (state_ != State::LISTENING) {
            modem_.setListen(true);
        }
        state_ = State::LISTENING;
        emitOK();
        return;
    }

    if (state_ == State::LISTENING) {
        pending_inbound_ = false;
        modem_.setListen(false);
    }
    state_ = State::READY;
    emitOK();
}

void TNCSession::cmdConnect(std::string_view args) {
    if (mycall_.empty() || state_ == State::IDLE || state_ == State::CONNECTING ||
        state_ == State::CONNECTED || state_ == State::DISCONNECTING) {
        emitWrong();
        return;
    }

    auto calls = parseCallsignList(args);
    if (calls.size() != 2 || !isValidCallsignToken(calls[0]) || !isValidCallsignToken(calls[1])) {
        emitWrong();
        return;
    }

    if (state_ == State::LISTENING) {
        modem_.setListen(false);
    }
    pending_inbound_ = false;
    state_ = State::CONNECTING;
    modem_.startConnect(calls[0], calls[1]);
    emitOK();
}

void TNCSession::cmdDisconnect() {
    // DISCONNECT during the call setup cancels it (it used to get WRONG and the session then
    // completed anyway, GitHub #38). The bridge reports DISCONNECTED when the link is down.
    if (state_ == State::CONNECTING) {
        modem_.abort();
        pending_inbound_ = false;
        state_ = isReadyAfterDisconnect(mycall_) ? State::READY : State::IDLE;
        emitOK();
        return;
    }
    if (state_ != State::CONNECTED && state_ != State::DISCONNECTING) {
        emitWrong();
        return;
    }

    if (state_ == State::CONNECTED) {
        state_ = State::DISCONNECTING;
        modem_.disconnect();
    }
    emitOK();
}

void TNCSession::cmdAbort() {
    if (state_ != State::CONNECTING && state_ != State::CONNECTED && state_ != State::DISCONNECTING) {
        emitWrong();
        return;
    }

    modem_.abort();
    pending_inbound_ = false;
    state_ = isReadyAfterDisconnect(mycall_) ? State::READY : State::IDLE;
    emitOK();
}

void TNCSession::cmdCompression(std::string_view args) {
    const auto tokens = splitWhitespace(args);
    if (tokens.size() > 1) {
        emitWrong();
        return;
    }

    if (tokens.empty()) {
        compression_enabled_ = false;
        emitOK();
        return;
    }

    const std::string mode = toUpper(tokens.front());
    if (mode != "OFF" && mode != "TEXT" && mode != "FILES" && mode != "ON") {
        emitWrong();
        return;
    }

    compression_enabled_ = mode != "OFF";
    emitOK();
}

void TNCSession::cmdChat(std::string_view args) {
    const auto tokens = splitWhitespace(args);
    if (tokens.size() != 1) {
        emitWrong();
        return;
    }

    const std::string mode = toUpper(tokens.front());
    if (mode == "ON") {
        chat_enabled_ = true;
        emitOK();
    } else if (mode == "OFF") {
        chat_enabled_ = false;
        emitOK();
    } else {
        emitWrong();
    }
}

void TNCSession::cmdCwid(std::string_view args) {
    const auto tokens = splitWhitespace(args);
    if (tokens.size() != 1) {
        emitWrong();
        return;
    }

    const std::string mode = toUpper(tokens.front());
    if (mode == "ON") {
        cwid_enabled_ = true;
        emitOK();
    } else if (mode == "OFF") {
        cwid_enabled_ = false;
        emitOK();
    } else {
        emitWrong();
    }
}

void TNCSession::cmdVersion(std::string_view args) {
    if (!trimView(args).empty()) {
        emitWrong();
        return;
    }
    emitVersion();
}

void TNCSession::cmdBuffer(std::string_view args) {
    if (!trimView(args).empty()) {
        emitWrong();
        return;
    }
    // Match the same accounting as the unsolicited BUFFER N event:
    // include both engine backlog and our local TX staging buffer.
    emitBuffer(modem_.getTxBackloggBytes() +
               static_cast<int>(data_tx_buffer_.size()));
}

void TNCSession::cmdSn(std::string_view args) {
    if (!trimView(args).empty()) {
        emitWrong();
        return;
    }
    last_snr_db_ = static_cast<float>(modem_.getCurrentSNR_db());
    emitSN(last_snr_db_);
}

void TNCSession::cmdBitrate(std::string_view args) {
    if (!trimView(args).empty()) {
        emitWrong();
        return;
    }
    last_bitrate_bps_ = modem_.getCurrentBitrate_bps();
    emitBitrate(last_bitrate_bps_);
}

void TNCSession::cmdPublic(std::string_view args) {
    const auto tokens = splitWhitespace(args);
    if (tokens.size() != 1) {
        emitWrong();
        return;
    }

    const std::string mode = toUpper(tokens.front());
    if (mode == "ON" || mode == "OFF") {
        emitOK();
    } else {
        emitWrong();
    }
}

void TNCSession::cmdP2P(std::string_view args) {
    const auto tokens = splitWhitespace(args);
    if (tokens.empty() || (tokens.size() == 1 && toUpper(tokens.front()) == "SESSION")) {
        emitOK();
    } else {
        emitWrong();
    }
}

void TNCSession::cmdWinlink(std::string_view args) {
    const auto tokens = splitWhitespace(args);
    if (tokens.empty() || (tokens.size() == 1 && toUpper(tokens.front()) == "SESSION")) {
        emitOK();
    } else {
        emitWrong();
    }
}

void TNCSession::cmdIgnoreKissDcd(std::string_view args) {
    (void)args;
    emitOK();
}

void TNCSession::cmdIntegerNoop(std::string_view args) {
    int value = 0;
    if (parseNonNegativeInt(args, value)) {
        emitOK();
    } else {
        emitWrong();
    }
}

void TNCSession::cmdStats(std::string_view args) {
    if (!trimView(args).empty()) {
        emitWrong();
        return;
    }

    const ModemStats s = modem_.getStats();
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "STATS frames_sent=%d frames_recv=%d retx=%d timeouts=%d "
                  "failed=%d out_of_order=%d rate=%s mod=%s mode=%s snr=%d "
                  "snr_source=%s bps=%d backlog=%d\r",
                  s.frames_sent, s.frames_received, s.retransmissions,
                  s.timeouts, s.failed, s.out_of_order,
                  s.code_rate.empty() ? "?" : s.code_rate.c_str(),
                  s.modulation.empty() ? "?" : s.modulation.c_str(),
                  s.waveform.empty() ? "?" : s.waveform.c_str(),
                  s.snr_db, s.snr_source.empty() ? "none" : s.snr_source.c_str(),
                  s.bitrate_bps, s.tx_backlog_bytes);
    if (cmd_emit_) {
        cmd_emit_(buf);
    }
}

} // namespace ultra::tnc
