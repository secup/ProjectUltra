#pragma once

#include "ultra/types.hpp"
#include <deque>
#include <fstream>
#include <functional>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <cstdint>

namespace ultra {
namespace protocol {

// Payload type discriminator (first byte of DATA frame payload)
enum class PayloadType : uint8_t {
    TEXT_MESSAGE = 0x00,  // Legacy bare text object (no identity) — RX-only
    FILE_START   = 0x01,  // File metadata: flags, size, crc32, filename
    FILE_DATA    = 0x02,  // File chunk: offset + data
    FILE_BLOCK   = 0x03,  // Single-frame file block: metadata + full payload
    // Text message object carrying a logical identity: {0x04, object_id, text...}.
    // The id makes a re-send IDEMPOTENT, which is what lets a mandatory geometry
    // escape re-fragment an in-flight message instead of destroying it
    // (BUG-MESSAGE-LOST-ON-FORCED-DEMOTE). The sender cannot know whether the peer
    // already holds the object — the classic ARQ ambiguity — so the receiver, which
    // CAN know, drops the duplicate.
    //
    // Deliberately NOT in the fixed-frame header: HEADER_SIZE feeds
    // FIXED_FRAME_OVERHEAD, CONNECT sizing and the RX framer, so a header byte would
    // ripple into control frames and capacity math. This prefix is parsed only by the
    // message reassembly path.
    TEXT_MESSAGE_OBJECT = 0x04,
};

// {type, object_id} — see PayloadType::TEXT_MESSAGE_OBJECT.
constexpr size_t kMessageObjectPrefixBytes = 2;

// File transfer flags (in FILE_START payload)
namespace FileFlags {
    constexpr uint8_t NONE       = 0x00;
    constexpr uint8_t COMPRESSED = 0x01;  // File data is deflate-compressed
}

// File transfer progress information
struct FileTransferProgress {
    std::string filename;
    uint32_t total_bytes = 0;
    uint32_t transferred_bytes = 0;   // RX: contiguous bytes safely assembled to file
    uint32_t received_bytes = 0;      // RX: ALL unique bytes received incl. out-of-order
                                      //     (SR-ARQ buffered, awaiting an earlier gap to fill).
                                      //     TX: == transferred_bytes.
    bool is_sending = false;

    // Contiguous (file-safe) progress — what is actually written/assembled in order.
    float percentage() const {
        return total_bytes > 0 ? (100.0f * transferred_bytes / total_bytes) : 0.0f;
    }
    // True received progress — contiguous + out-of-order buffered. >= percentage();
    // the gap between them is the SR-ARQ recovery currently in flight.
    float receivedPercentage() const {
        return total_bytes > 0 ? (100.0f * received_bytes / total_bytes) : 0.0f;
    }
};

// File transfer state
enum class FileTransferState {
    IDLE,
    SENDING,
    RECEIVING,
    COMPLETE,
    ERROR
};

/**
 * FileTransferController
 *
 * Handles chunking files for transmission and reassembling received chunks.
 * Integrates with the ARQ layer for reliable delivery.
 *
 * TX Flow:
 *   1. Call startSend(filepath) to begin
 *   2. Call getNextChunk() to get each payload (includes type byte)
 *   3. Call onChunkAcked() when ARQ confirms delivery
 *   4. Repeat until hasMoreChunks() returns false
 *
 * RX Flow:
 *   1. Call processPayload() for each received DATA frame
 *   2. Returns true if it was a file transfer frame
 *   3. On completion, on_received_ callback is invoked
 */
class FileTransferController {
public:
    // Default max data per chunk (256 - 1 type - 4 offset - 1 safety margin)
    // For OFDM fixed frames, call setMaxChunkPayload() to match frame capacity
    static constexpr size_t DEFAULT_CHUNK_SIZE = 250;
    static constexpr size_t FILE_DATA_OVERHEAD = 5;  // TYPE(1) + OFFSET(4)
    static constexpr size_t FILE_START_FIXED_OVERHEAD = 10;  // TYPE+FLAGS+SIZE+CRC
    // FILE_START has no explicit filename length; at least one filename byte is
    // required because the receiver rejects payloads shorter than 11 bytes.
    static constexpr size_t MIN_FILE_START_PAYLOAD = FILE_START_FIXED_OVERHEAD + 1;
    static constexpr size_t FILE_BLOCK_FIXED_OVERHEAD = 15;  // TYPE+FLAGS+SIZE+TX_SIZE+CRC+NAME_LEN

    // Callbacks
    using ProgressCallback = std::function<void(const FileTransferProgress&)>;
    using ReceivedCallback = std::function<void(const std::string& path, bool success,
                                                const std::string& error)>;
    using SentCallback = std::function<void(bool success, const std::string& error)>;

    FileTransferController() = default;
    ~FileTransferController();

    // Set max payload per ARQ frame (call before startSend).
    // Chunk data size = max_payload - FILE_DATA_OVERHEAD (5 bytes for type+offset)
    void setMaxChunkPayload(size_t max_payload) {
        max_chunk_payload_ = max_payload;
        if (max_payload > FILE_DATA_OVERHEAD) {
            chunk_size_ = max_payload - FILE_DATA_OVERHEAD;
        }
    }

    // --- TX Side ---

    // Start sending a file. Returns false if busy or file not found.
    bool startSend(const std::string& filepath);

    // Get the next chunk payload (includes type byte).
    // Returns empty if nothing to send.
    Bytes getNextChunk();

    // Build a complete single-frame file payload if it fits max_payload.
    // Returns empty when the file must use the chunked FILE_START/FILE_DATA path.
    Bytes getSingleBlockPayload(size_t max_payload);

    // Check if there are more chunks to send
    bool hasMoreChunks() const;

    // Check if there are chunks sent but not yet ACKed
    bool hasPendingChunks() const { return chunks_sent_ > chunks_acked_; }
    uint32_t pendingChunkCount() const {
        return chunks_sent_ > chunks_acked_ ? chunks_sent_ - chunks_acked_ : 0;
    }
    size_t remainingTxBytes() const {
        if (state_ != FileTransferState::SENDING || tx_offset_ >= tx_data_.size()) {
            return 0;
        }
        return tx_data_.size() - tx_offset_;
    }

    // Rewind queued-but-unACKed chunks so ARQ can re-emit them after it drops
    // in-flight frames, for example across a fixed-frame code-rate change.
    // Call before changing max chunk payload so the old chunk size is used to
    // find the first unACKed byte offset.
    void requeuePendingChunks();

    // Called when the current chunk is ACKed
    void onChunkAcked();

    // Called when send fails (max retries exceeded)
    void onSendFailed();

    // --- RX Side ---

    // Process received DATA payload (already stripped of frame header).
    // Returns true if this was a file transfer frame (vs text message).
    // more_data indicates if MORE_DATA flag was set on the frame.
    bool processPayload(const Bytes& payload, bool more_data);

    // Set directory for received files
    void setReceiveDirectory(const std::string& dir);

    // --- State ---

    FileTransferState getState() const { return state_; }
    FileTransferProgress getProgress() const;
    bool isBusy() const;

    // True when decoded FILE_DATA was safely STAGED ahead of FILE_START (and within
    // bounds). The connection may then ACK those frames instead of NACKing the whole
    // burst — staging guarantees they survive until FILE_START drains them, so we are
    // never ACKing data that gets silently dropped.
    bool hasHealthyStagedData() const {
        return !rx_prestart_chunks_.empty() && !rx_prestart_overflow_;
    }

    // Cancel current transfer
    void cancel(const std::string& error = "Transfer cancelled");

    // --- Callbacks ---

    void setProgressCallback(ProgressCallback cb) { on_progress_ = cb; }
    void setReceivedCallback(ReceivedCallback cb) { on_received_ = cb; }
    void setSentCallback(SentCallback cb) { on_sent_ = cb; }

    // F163 FIX-4: mark [offset, offset+len) as PEER-CONFIRMED delivered (a SACKed
    // frame a rate-change abort discarded). getNextChunkPayload skips these
    // ranges instead of re-sending bytes the receiver already holds; the
    // receiver's offset-idempotent reassembly makes over-sending harmless but
    // wasteful. Ranges live for the current TX file only.
    void noteRangeDelivered(uint32_t offset, uint32_t len);

    // F218 COMPLETION GATE: chunk-count completion has produced THREE false
    // "Transfer complete" wedges (F168 receiver stranded at 50 %, F181 phantom
    // -SACK first-chunk skip, F218 complete with 10 frames in flight — sender
    // idle, receiver stuck 93 %/88 %). Counts drift across re-encodes; the
    // ground truth is the ARQ: NOTHING in flight. The host injects the gate;
    // completion defers until it returns true and is re-checked via
    // maybeCompleteSend() on ack/base-advance events.
    void setCompletionGate(std::function<bool()> gate) {
        completion_gate_ = std::move(gate);
    }
    void maybeCompleteSend();

private:
    FileTransferState state_ = FileTransferState::IDLE;
    size_t chunk_size_ = DEFAULT_CHUNK_SIZE;
    // Keep the physical payload bound separately from FILE_DATA's usable bytes.
    // A profile may carry FILE_DATA (>=6 B) yet be too small for FILE_START (<11 B).
    // Recording it lets startSend fail cleanly without disabling an already-started
    // transfer that later moves to a small but still usable FILE_DATA geometry.
    size_t max_chunk_payload_ = DEFAULT_CHUNK_SIZE + FILE_DATA_OVERHEAD;
    std::function<bool()> completion_gate_;  // F218: ARQ-idle required

    void skipDeliveredRanges();  // F163 FIX-4

    // TX state
    std::vector<std::pair<uint32_t, uint32_t>> tx_delivered_ranges_;  // F163 FIX-4 (offset,len)
    std::string tx_filepath_;
    std::string tx_filename_;  // Just the filename, no path
    Bytes tx_data_;            // File data (possibly compressed)
    uint32_t tx_original_size_ = 0;  // Original uncompressed size
    uint32_t tx_crc_ = 0;      // CRC32 of original file
    uint32_t tx_offset_ = 0;   // Current offset in tx_data_
    uint8_t tx_flags_ = 0;     // FileFlags
    bool tx_metadata_sent_ = false;
    uint32_t chunks_sent_ = 0;      // Chunks queued to ARQ (for window tracking)
    uint32_t chunks_acked_ = 0;     // Chunks confirmed by ARQ

    // Send-order ledger of chunks handed to the ARQ and not yet retired.
    // ARQ retirement (onChunkAcked) happens strictly in send order (TX-base
    // advance), so front() is always the oldest un-retired chunk and
    // requeuePendingChunks() can resume exactly at front().offset. chunk_size_
    // changes on every mid-stream rate/mod move, so the acked-chunk history is
    // heterogeneous — any count*chunk_size_ reconstruction of the resume offset
    // is wrong once the size has ever changed (it skipped 10 KB forward on the
    // Moderate@20 ladder cell).
    struct PendingTxChunk {
        uint32_t offset = 0;
        bool metadata = false;  // FILE_START / single block: requeue rebuilds from scratch
    };
    std::deque<PendingTxChunk> tx_pending_ledger_;

    // RX state
    std::string rx_dir_ = ".";
    std::string rx_filepath_;
    std::string rx_filename_;
    Bytes rx_data_;            // Accumulated data (possibly compressed)
    uint32_t rx_expected_size_ = 0;  // Original uncompressed size
    uint32_t rx_expected_crc_ = 0;
    uint8_t rx_flags_ = 0;     // FileFlags from FILE_START
    bool rx_final_chunk_seen_ = false;
    std::map<uint32_t, Bytes> rx_pending_chunks_;  // Out-of-order chunks buffered by offset

    // FILE_DATA decoded BEFORE FILE_START established RECEIVING. The BURST_HEADER has
    // already announced an inbound bulk burst, and every FILE_DATA frame carries its
    // own ABSOLUTE byte offset + length, so we STAGE these (keyed by offset) instead
    // of dropping them, then DRAIN them when FILE_START arrives — saving a re-send of
    // frames that already decoded. ADAPTIVE BY CONSTRUCTION: the staging is purely
    // offset/length based and reads NOTHING from the BURST_HEADER (not group size, cw,
    // mod, rate, or Z), so it stays correct if the descriptor changes between/within
    // bursts (e.g. adaptive rate). Bounded: on overflow we stop staging and the
    // connection falls back to NACK, so we never ACK data we cannot retain.
    std::map<uint32_t, Bytes> rx_prestart_chunks_;
    bool rx_prestart_final_seen_ = false;
    bool rx_prestart_overflow_ = false;
    static constexpr size_t MAX_PRESTART_STAGE_BYTES = 256u * 1024u;

    // Callbacks
    ProgressCallback on_progress_;
    ReceivedCallback on_received_;
    SentCallback on_sent_;

    // Helpers
    Bytes buildMetadataPayload();
    Bytes buildDataPayload();
    Bytes buildSingleBlockPayload(size_t max_payload);
    bool processFileStart(const Bytes& payload);
    bool processFileData(const Bytes& payload, bool more_data);
    void drainPendingChunks();  // overlap-aware: covered entries drop, straddlers tail-append
    bool processFileBlock(const Bytes& payload);
    void checkAndFinalizeReceive();  // CRC-check + write + callback once data is complete
    uint32_t calculateCRC32(std::istream& stream);
    uint32_t calculateCRC32File(const std::string& filepath);
    void resetTxState();
    void resetRxState();
    void notifyProgress();

    // Extract just the filename from a path
    static std::string extractFilename(const std::string& path);
};

} // namespace protocol
} // namespace ultra
