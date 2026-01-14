#pragma once

#include "ultra/types.hpp"
#include <fstream>
#include <functional>
#include <string>
#include <cstdint>

namespace ultra {
namespace protocol {

// Payload type discriminator (first byte of DATA frame payload)
enum class PayloadType : uint8_t {
    TEXT_MESSAGE = 0x00,  // Regular text message (backward compatible)
    FILE_START   = 0x01,  // File metadata: flags, size, crc32, filename
    FILE_DATA    = 0x02,  // File chunk: offset + data
};

// File transfer flags (in FILE_START payload)
namespace FileFlags {
    constexpr uint8_t NONE       = 0x00;
    constexpr uint8_t COMPRESSED = 0x01;  // File data is deflate-compressed
}

// File transfer progress information
struct FileTransferProgress {
    std::string filename;
    uint32_t total_bytes = 0;
    uint32_t transferred_bytes = 0;
    bool is_sending = false;

    float percentage() const {
        return total_bytes > 0 ? (100.0f * transferred_bytes / total_bytes) : 0.0f;
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
    // Maximum data per chunk (256 - 1 type - 4 offset - 1 safety margin)
    static constexpr size_t CHUNK_SIZE = 250;

    // Callbacks
    using ProgressCallback = std::function<void(const FileTransferProgress&)>;
    using ReceivedCallback = std::function<void(const std::string& path, bool success)>;
    using SentCallback = std::function<void(bool success, const std::string& error)>;

    FileTransferController() = default;
    ~FileTransferController();

    // --- TX Side ---

    // Start sending a file. Returns false if busy or file not found.
    bool startSend(const std::string& filepath);

    // Get the next chunk payload (includes type byte).
    // Returns empty if nothing to send.
    Bytes getNextChunk();

    // Check if there are more chunks to send
    bool hasMoreChunks() const;

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

    // Cancel current transfer
    void cancel();

    // --- Callbacks ---

    void setProgressCallback(ProgressCallback cb) { on_progress_ = cb; }
    void setReceivedCallback(ReceivedCallback cb) { on_received_ = cb; }
    void setSentCallback(SentCallback cb) { on_sent_ = cb; }

private:
    FileTransferState state_ = FileTransferState::IDLE;

    // TX state
    std::string tx_filepath_;
    std::string tx_filename_;  // Just the filename, no path
    Bytes tx_data_;            // File data (possibly compressed)
    uint32_t tx_original_size_ = 0;  // Original uncompressed size
    uint32_t tx_crc_ = 0;      // CRC32 of original file
    uint32_t tx_offset_ = 0;   // Current offset in tx_data_
    uint8_t tx_flags_ = 0;     // FileFlags
    bool tx_metadata_sent_ = false;
    bool tx_waiting_ack_ = false;

    // RX state
    std::string rx_dir_ = ".";
    std::string rx_filepath_;
    std::string rx_filename_;
    Bytes rx_data_;            // Accumulated data (possibly compressed)
    uint32_t rx_expected_size_ = 0;  // Original uncompressed size
    uint32_t rx_expected_crc_ = 0;
    uint8_t rx_flags_ = 0;     // FileFlags from FILE_START

    // Callbacks
    ProgressCallback on_progress_;
    ReceivedCallback on_received_;
    SentCallback on_sent_;

    // Helpers
    Bytes buildMetadataPayload();
    Bytes buildDataPayload();
    bool processFileStart(const Bytes& payload);
    bool processFileData(const Bytes& payload, bool more_data);
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
