#include "file_transfer.hpp"
#include <atomic>
#include "compression.hpp"
#include "ultra/logging.hpp"
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <system_error>

namespace ultra {
namespace protocol {

namespace {

// CRC32 lookup table (polynomial 0xEDB88320)
constexpr uint32_t CRC32_TABLE[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
    0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
    0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
    0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
    0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
    0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55,
    0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28,
    0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
    0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
    0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
    0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
    0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693,
    0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

uint32_t updateCRC32(uint32_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        crc = CRC32_TABLE[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc;
}

std::string sanitizeReceivedFilename(std::string filename) {
    for (char& c : filename) {
        unsigned char ch = static_cast<unsigned char>(c);
        if (c == '/' || c == '\\' || c == ':' || ch < 0x20) {
            c = '_';
        }
    }

    if (filename.empty() || filename == "." || filename == "..") {
        return "received_file";
    }

    return filename;
}

std::string uniqueReceivePath(const std::string& directory, const std::string& filename) {
    const std::filesystem::path dir(directory);
    const std::filesystem::path name(filename);
    std::filesystem::path candidate = dir / name;

    std::error_code ec;
    if (!std::filesystem::exists(candidate, ec)) {
        return candidate.string();
    }

    const std::string stem = name.stem().string().empty()
                                 ? std::string("received_file")
                                 : name.stem().string();
    const std::string extension = name.extension().string();

    for (int suffix = 1; suffix < 100000; ++suffix) {
        candidate = dir / (stem + "_" + std::to_string(suffix) + extension);
        ec.clear();
        if (!std::filesystem::exists(candidate, ec)) {
            return candidate.string();
        }
    }

    return (dir / (stem + "_overflow" + extension)).string();
}

} // anonymous namespace

FileTransferController::~FileTransferController() {
    cancel();
}

bool FileTransferController::startSend(const std::string& filepath) {
    if (isBusy() || max_chunk_payload_ < MIN_FILE_START_PAYLOAD) {
        return false;
    }

    // Read entire file into memory
    std::ifstream file(filepath, std::ios::binary | std::ios::ate);
    if (!file.good()) {
        return false;
    }

    auto size = file.tellg();
    if (size < 0 || static_cast<uint64_t>(size) > UINT32_MAX) {
        return false;  // File too large
    }

    file.seekg(0, std::ios::beg);
    Bytes file_data(static_cast<size_t>(size));
    file.read(reinterpret_cast<char*>(file_data.data()), size);
    file.close();

    // Calculate CRC32 of original data
    uint32_t crc = 0xFFFFFFFF;
    crc = updateCRC32(crc, file_data.data(), file_data.size());
    tx_crc_ = crc ^ 0xFFFFFFFF;

    tx_original_size_ = static_cast<uint32_t>(file_data.size());
    tx_filepath_ = filepath;
    tx_filename_ = extractFilename(filepath);

    // Try to compress if beneficial
    tx_flags_ = FileFlags::NONE;
    if (Compression::shouldCompress(file_data)) {
        auto compressed = Compression::compress(file_data);
        if (compressed && compressed->size() < file_data.size()) {
            tx_data_ = std::move(*compressed);
            tx_flags_ = FileFlags::COMPRESSED;
        } else {
            tx_data_ = std::move(file_data);
        }
    } else {
        tx_data_ = std::move(file_data);
    }

    tx_offset_ = 0;
    tx_metadata_sent_ = false;
    chunks_sent_ = 0;
    chunks_acked_ = 0;
    // An inbound FILE_START can trample SENDING->RECEIVING without resetTxState,
    // orphaning ledger entries that would poison this transfer's requeue offsets.
    tx_pending_ledger_.clear();
    state_ = FileTransferState::SENDING;

    notifyProgress();
    return true;
}

Bytes FileTransferController::getNextChunk() {
    if (state_ != FileTransferState::SENDING) {
        return {};
    }

    // Note: We no longer block on tx_waiting_ack_ since Selective Repeat ARQ
    // handles multiple frames in flight. The ARQ window controls pacing.

    Bytes payload;
    bool is_metadata = false;
    uint32_t chunk_offset = 0;

    if (!tx_metadata_sent_) {
        payload = buildMetadataPayload();
        if (!payload.empty()) {
            tx_metadata_sent_ = true;  // Mark sent immediately (ARQ handles retries)
            is_metadata = true;
        }
    } else if (tx_offset_ < tx_data_.size()) {
        chunk_offset = tx_offset_;  // buildDataPayload advances tx_offset_
        payload = buildDataPayload();
    }

    if (!payload.empty()) {
        chunks_sent_++;
        tx_pending_ledger_.push_back({chunk_offset, is_metadata});
    }

    return payload;
}

bool FileTransferController::hasMoreChunks() const {
    if (state_ != FileTransferState::SENDING) {
        return false;
    }
    return !tx_metadata_sent_ || tx_offset_ < tx_data_.size();
}

void FileTransferController::noteRangeDelivered(uint32_t offset, uint32_t len) {
    if (state_ != FileTransferState::SENDING || len == 0) {
        return;
    }
    tx_delivered_ranges_.emplace_back(offset, len);
    LOG_MODEM(INFO,
              "FileTransfer: range [%u, %u) peer-confirmed across abort — will "
              "skip on requeue",
              offset, offset + len);
}

// F163 FIX-4: advance tx_offset_ past any peer-confirmed range it sits inside.
void FileTransferController::skipDeliveredRanges() {
    bool moved = true;
    while (moved) {
        moved = false;
        for (const auto& r : tx_delivered_ranges_) {
            if (tx_offset_ >= r.first && tx_offset_ < r.first + r.second) {
                tx_offset_ = r.first + r.second;
                moved = true;
            }
        }
    }
    if (tx_offset_ > tx_data_.size()) {
        tx_offset_ = static_cast<uint32_t>(tx_data_.size());
    }
}

void FileTransferController::requeuePendingChunks() {
    if (state_ != FileTransferState::SENDING) {
        return;
    }

    if (chunks_sent_ <= chunks_acked_) {
        return;
    }

    const uint32_t pending = chunks_sent_ - chunks_acked_;
    // Price the rewind in BYTES, not chunks: `pending` over-counts the true waste
    // by ~3x because most re-queued chunks are genuine holes that had to be re-sent
    // anyway. requeued_bytes = the exact span this rewind will put back on the air,
    // which is the only number that can settle whether interior-skip is worth
    // building. Measurement only — no behaviour change.
    const uint32_t frontier = tx_offset_;
    if (tx_pending_ledger_.empty() || tx_pending_ledger_.front().metadata) {
        // Empty ledger with pending counts means the count/ledger bookkeeping
        // diverged (identity-blind acks) — restart from scratch. A rewind is
        // always SAFE (FILE_DATA carries explicit offsets, duplicates are
        // idempotent at the receiver); a forward jump silently loses bytes.
        tx_metadata_sent_ = false;
        tx_offset_ = 0;
    } else {
        tx_metadata_sent_ = true;
        tx_offset_ = tx_pending_ledger_.front().offset;
    }

    tx_pending_ledger_.clear();
    chunks_sent_ = chunks_acked_;
    skipDeliveredRanges();  // F163 FIX-4: don't re-send peer-confirmed bytes

    const uint32_t requeued_bytes = (frontier > tx_offset_) ? (frontier - tx_offset_) : 0u;
    LOG_MODEM(WARN,
              "FileTransfer: Re-queued %u pending chunks after ARQ abort (acked=%u, "
              "resume_offset=%u, requeued_bytes=%u)",
              pending, chunks_acked_, tx_offset_, requeued_bytes);
    notifyProgress();
}

void FileTransferController::onChunkAcked() {
    if (state_ != FileTransferState::SENDING) {
        return;
    }

    // Track how many chunks have been ACKed for progress
    chunks_acked_++;
    if (!tx_pending_ledger_.empty()) {
        tx_pending_ledger_.pop_front();  // retirement is in send order
    }

    notifyProgress();
    maybeCompleteSend();
}

void FileTransferController::maybeCompleteSend() {
    if (state_ != FileTransferState::SENDING) {
        return;
    }
    if (hasMoreChunks() || hasPendingChunks()) {
        return;
    }
    // F218 COMPLETION GATE: the chunk ledger says done, but the ARQ is the
    // ground truth — never declare complete while frames are in flight (the
    // ledger has drifted three separate ways across re-encodes; each time the
    // sender went idle and stranded the receiver's holes). Deferring keeps
    // state SENDING, so ARQ RTOs keep retransmitting the outstanding frames;
    // the host re-invokes this on ack/base-advance events until the flight
    // drains.
    if (completion_gate_ && !completion_gate_()) {
        static std::atomic<int> defer_log{0};
        if (++defer_log % 5 == 1) {
            LOG_MODEM(WARN,
                      "FileTransfer: chunk ledger complete but ARQ still has "
                      "frames in flight — completion DEFERRED (F218 gate)");
        }
        return;
    }
    const auto sent_callback = on_sent_;
    // Retire the completed transfer before entering application code. A success
    // callback may synchronously start the next file; resetting afterward would
    // wipe that replacement's path, bytes and chunk ledger.
    resetTxState();
    if (sent_callback) {
        sent_callback(true, "");
    }
}

void FileTransferController::onSendFailed() {
    if (state_ != FileTransferState::SENDING) {
        return;
    }

    state_ = FileTransferState::ERROR;
    const auto sent_callback = on_sent_;
    // Clear the failed transfer before entering application code. A failure callback
    // is allowed to start another transfer; resetting after that callback would wipe
    // the newly loaded file and leave its state/ledger inconsistent.
    resetTxState();
    if (sent_callback) {
        sent_callback(false, "Transfer failed: max retries exceeded");
    }
}

bool FileTransferController::processPayload(const Bytes& payload, bool more_data) {
    if (payload.empty()) {
        return false;
    }

    PayloadType type = static_cast<PayloadType>(payload[0]);

    switch (type) {
        case PayloadType::FILE_START:
            return processFileStart(payload);

        case PayloadType::FILE_DATA:
            return processFileData(payload, more_data);

        case PayloadType::FILE_BLOCK:
            return processFileBlock(payload);

        case PayloadType::TEXT_MESSAGE:
        default:
            return false;
    }
}

void FileTransferController::setReceiveDirectory(const std::string& dir) {
    rx_dir_ = dir;
    try {
        std::filesystem::create_directories(dir);
    } catch (...) {
    }
}

FileTransferProgress FileTransferController::getProgress() const {
    FileTransferProgress progress;

    if (state_ == FileTransferState::SENDING) {
        progress.filename = tx_filename_;
        progress.total_bytes = static_cast<uint32_t>(tx_data_.size());
        progress.transferred_bytes = tx_offset_;
        progress.received_bytes = tx_offset_;
        progress.is_sending = true;
    } else if (state_ == FileTransferState::RECEIVING) {
        progress.filename = rx_filename_;
        progress.total_bytes = rx_expected_size_;
        progress.transferred_bytes = static_cast<uint32_t>(rx_data_.size());
        // True received total = contiguous + every out-of-order chunk still buffered
        // (SR-ARQ keeps these; they drain into rx_data_ when the earlier gap fills).
        // No double-count: draining moves bytes pending->rx_data_, sum is invariant.
        uint64_t buffered = 0;
        for (const auto& kv : rx_pending_chunks_) buffered += kv.second.size();
        progress.received_bytes =
            static_cast<uint32_t>(rx_data_.size() + buffered);
        progress.is_sending = false;
    }

    return progress;
}

bool FileTransferController::isBusy() const {
    return state_ == FileTransferState::SENDING ||
           state_ == FileTransferState::RECEIVING;
}

void FileTransferController::cancel(const std::string& error) {
    if (state_ == FileTransferState::SENDING) {
        if (on_sent_) {
            on_sent_(false, error);
        }
        resetTxState();
    } else if (state_ == FileTransferState::RECEIVING) {
        if (on_received_) {
            on_received_("", false, error);
        }
        resetRxState();
    }

    state_ = FileTransferState::IDLE;
}

// --- Private Methods ---

Bytes FileTransferController::buildMetadataPayload() {
    // Format: TYPE(1) + FLAGS(1) + ORIGINAL_SIZE(4) + CRC32(4) + FILENAME(variable)
    // startSend() rejects this geometry, but keep this private builder fail-closed
    // too: metadata may be requeued after a mid-transfer geometry change.
    if (max_chunk_payload_ < MIN_FILE_START_PAYLOAD) {
        return {};
    }

    const size_t max_name_len = max_chunk_payload_ - FILE_START_FIXED_OVERHEAD;
    const size_t name_len = std::min(tx_filename_.size(), max_name_len);
    Bytes payload;
    payload.reserve(FILE_START_FIXED_OVERHEAD + name_len);

    // Type byte
    payload.push_back(static_cast<uint8_t>(PayloadType::FILE_START));

    // Flags byte
    payload.push_back(tx_flags_);

    // Original size (big-endian)
    payload.push_back((tx_original_size_ >> 24) & 0xFF);
    payload.push_back((tx_original_size_ >> 16) & 0xFF);
    payload.push_back((tx_original_size_ >> 8) & 0xFF);
    payload.push_back(tx_original_size_ & 0xFF);

    // CRC32 of original file (big-endian)
    payload.push_back((tx_crc_ >> 24) & 0xFF);
    payload.push_back((tx_crc_ >> 16) & 0xFF);
    payload.push_back((tx_crc_ >> 8) & 0xFF);
    payload.push_back(tx_crc_ & 0xFF);

    // Filename (truncate if too long to fit in one frame)
    // Metadata overhead: TYPE(1) + FLAGS(1) + SIZE(4) + CRC32(4) = 10 bytes
    payload.insert(payload.end(), tx_filename_.begin(), tx_filename_.begin() + name_len);

    return payload;
}

Bytes FileTransferController::buildDataPayload() {
    // Format: TYPE(1) + OFFSET(4) + DATA(up to CHUNK_SIZE)
    Bytes payload;

    payload.push_back(static_cast<uint8_t>(PayloadType::FILE_DATA));

    // Offset in transmitted data (big-endian)
    payload.push_back((tx_offset_ >> 24) & 0xFF);
    payload.push_back((tx_offset_ >> 16) & 0xFF);
    payload.push_back((tx_offset_ >> 8) & 0xFF);
    payload.push_back(tx_offset_ & 0xFF);

    // Data chunk
    size_t remaining = tx_data_.size() - tx_offset_;
    size_t chunk_size = std::min(remaining, chunk_size_);
    // F163 FIX-4: never slice INTO a peer-confirmed range — cap at its start
    // (the skip below then jumps the offset past it on the next chunk).
    for (const auto& r : tx_delivered_ranges_) {
        if (r.first > tx_offset_ && r.first < tx_offset_ + chunk_size) {
            chunk_size = r.first - tx_offset_;
        }
    }

    payload.insert(payload.end(),
                   tx_data_.begin() + tx_offset_,
                   tx_data_.begin() + tx_offset_ + chunk_size);

    tx_offset_ += static_cast<uint32_t>(chunk_size);
    skipDeliveredRanges();  // F163 FIX-4

    return payload;
}

Bytes FileTransferController::getSingleBlockPayload(size_t max_payload) {
    if (state_ != FileTransferState::SENDING || tx_metadata_sent_ || tx_offset_ != 0) {
        return {};
    }

    Bytes payload = buildSingleBlockPayload(max_payload);
    if (payload.empty()) {
        return {};
    }

    tx_metadata_sent_ = true;
    tx_offset_ = static_cast<uint32_t>(tx_data_.size());
    chunks_sent_++;
    tx_pending_ledger_.push_back({0, true});  // single block rebuilds from scratch on requeue
    notifyProgress();
    return payload;
}

Bytes FileTransferController::buildSingleBlockPayload(size_t max_payload) {
    if (tx_filename_.size() > 255) {
        return {};
    }

    const size_t needed = FILE_BLOCK_FIXED_OVERHEAD + tx_filename_.size() + tx_data_.size();
    if (needed > max_payload) {
        return {};
    }

    Bytes payload;
    payload.reserve(needed);
    payload.push_back(static_cast<uint8_t>(PayloadType::FILE_BLOCK));
    payload.push_back(tx_flags_);

    payload.push_back((tx_original_size_ >> 24) & 0xFF);
    payload.push_back((tx_original_size_ >> 16) & 0xFF);
    payload.push_back((tx_original_size_ >> 8) & 0xFF);
    payload.push_back(tx_original_size_ & 0xFF);

    const uint32_t transmitted_size = static_cast<uint32_t>(tx_data_.size());
    payload.push_back((transmitted_size >> 24) & 0xFF);
    payload.push_back((transmitted_size >> 16) & 0xFF);
    payload.push_back((transmitted_size >> 8) & 0xFF);
    payload.push_back(transmitted_size & 0xFF);

    payload.push_back((tx_crc_ >> 24) & 0xFF);
    payload.push_back((tx_crc_ >> 16) & 0xFF);
    payload.push_back((tx_crc_ >> 8) & 0xFF);
    payload.push_back(tx_crc_ & 0xFF);

    payload.push_back(static_cast<uint8_t>(tx_filename_.size()));
    payload.insert(payload.end(), tx_filename_.begin(), tx_filename_.end());
    payload.insert(payload.end(), tx_data_.begin(), tx_data_.end());
    return payload;
}

bool FileTransferController::processFileStart(const Bytes& payload) {
    // Format: TYPE(1) + FLAGS(1) + SIZE(4) + CRC32(4) + FILENAME
    if (payload.size() < 11) {
        return true;  // Malformed
    }

    // Idempotent: a re-sent FILE_START (e.g. its ACK was lost) must NOT wipe an
    // in-progress receive. Only the FIRST FILE_START sets up the buffers / drains the
    // pre-start staging. (CRC is validated at completion, so a stale duplicate here is
    // safely ignored.)
    if (state_ == FileTransferState::RECEIVING) {
        return true;
    }

    rx_flags_ = payload[1];

    // Parse original size (big-endian)
    rx_expected_size_ = (static_cast<uint32_t>(payload[2]) << 24) |
                        (static_cast<uint32_t>(payload[3]) << 16) |
                        (static_cast<uint32_t>(payload[4]) << 8) |
                        static_cast<uint32_t>(payload[5]);

    // Parse CRC32 (big-endian)
    rx_expected_crc_ = (static_cast<uint32_t>(payload[6]) << 24) |
                       (static_cast<uint32_t>(payload[7]) << 16) |
                       (static_cast<uint32_t>(payload[8]) << 8) |
                       static_cast<uint32_t>(payload[9]);

    rx_filename_ = sanitizeReceivedFilename(std::string(payload.begin() + 10, payload.end()));
    rx_filepath_ = uniqueReceivePath(rx_dir_, rx_filename_);

    rx_data_.clear();
    rx_data_.reserve(std::min<uint32_t>(rx_expected_size_, 1024u * 1024u));
    state_ = FileTransferState::RECEIVING;

    // DRAIN (do NOT clear) any FILE_DATA staged before this FILE_START. Those frames
    // already decoded (the BURST_HEADER told us a bulk burst was inbound) and carry
    // absolute offsets, so adopt them as the out-of-order buffer and stitch in whatever
    // is now contiguous from offset 0 — saving a re-send of already-decoded frames.
    // Offset-based, so it is correct regardless of the descriptor used to carry them.
    rx_pending_chunks_ = std::move(rx_prestart_chunks_);
    rx_prestart_chunks_.clear();
    rx_final_chunk_seen_ = rx_prestart_final_seen_;
    rx_prestart_final_seen_ = false;
    rx_prestart_overflow_ = false;
    drainPendingChunks();

    notifyProgress();
    // A tiny file whose final chunk was staged ahead of FILE_START is already complete
    // after the drain — finalize now instead of waiting for a data frame that won't come.
    checkAndFinalizeReceive();
    return true;
}

// Drain buffered out-of-order chunks against the contiguous edge. Overlap-aware:
// after a sender requeue resends on a CHANGED chunk grid (mid-stream rate/mod
// move), buffered old-grid entries can be fully covered by the advancing edge
// (drop — the bytes carry no new information) or straddle it (append only the
// unseen tail). An exact-offset-only drain strands covered entries forever,
// which permanently blocks the compressed finalization predicate
// (rx_pending_chunks_.empty()) even with every byte present.
void FileTransferController::drainPendingChunks() {
    while (!rx_pending_chunks_.empty()) {
        auto it = rx_pending_chunks_.begin();  // std::map: lowest offset first
        const size_t have = rx_data_.size();
        const uint32_t off = it->first;
        if (static_cast<size_t>(off) > have) {
            break;  // still a gap before the earliest buffered entry
        }
        const Bytes& buf = it->second;
        if (static_cast<size_t>(off) + buf.size() > have) {
            rx_data_.insert(rx_data_.end(), buf.begin() + (have - off), buf.end());
            LOG_MODEM(INFO,
                      "FileTransfer: Drained buffered chunk offset=%u len=%zu (total=%zu)",
                      off, buf.size(), rx_data_.size());
        } else {
            LOG_MODEM(DEBUG,
                      "FileTransfer: Dropped fully-covered buffered chunk offset=%u len=%zu (have=%zu)",
                      off, buf.size(), have);
        }
        rx_pending_chunks_.erase(it);
    }
}

bool FileTransferController::processFileData(const Bytes& payload, bool more_data) {
    if (payload.size() < 6) {
        return true;
    }

    // Parse 32-bit ABSOLUTE chunk offset (big-endian) + data (skip type+offset bytes).
    // The offset is self-describing per frame — independent of the BURST_HEADER's group
    // size / cw / mod / rate / Z — which is what makes both the out-of-order buffer and
    // the pre-FILE_START staging below adaptive to descriptor changes.
    uint32_t offset = (static_cast<uint32_t>(payload[1]) << 24) |
                      (static_cast<uint32_t>(payload[2]) << 16) |
                      (static_cast<uint32_t>(payload[3]) << 8) |
                      static_cast<uint32_t>(payload[4]);
    const uint8_t* data = payload.data() + 5;
    size_t data_len = payload.size() - 5;

    // PRE-START STAGING: this FILE_DATA decoded before FILE_START set up the transfer.
    // The BURST_HEADER already told us a bulk burst is inbound and the offset is
    // absolute, so KEEP it (drained when FILE_START arrives) instead of dropping it and
    // forcing a whole-group re-send. Bounded; on overflow we give up staging and let
    // the connection NACK so we never ACK data we cannot retain.
    if (state_ != FileTransferState::RECEIVING) {
        if (rx_prestart_overflow_) {
            return true;  // staging disabled this transfer; connection will NACK
        }
        size_t staged = 0;
        for (const auto& kv : rx_prestart_chunks_) staged += kv.second.size();
        // NEVER-SHRINK coverage (BUG-SACK-DURABILITY-RESIDUAL root cause, F181):
        // chunk_size_ changes on EVERY mid-stream rate/mod move, so the SAME byte
        // offset legitimately re-arrives with a SHORTER length on the new grid. The
        // bare `map[offset] = ...` this replaces then DESTROYED the tail of the longer
        // staged copy — F181: a 456 B chunk at offset 0 was overwritten by era-1's
        // re-gridded 408 B chunk, so bytes [408,456) ceased to exist; the sender's
        // range salvage then skipped re-sending them => permanently stranded file.
        // Offsets are absolute into ONE immutable tx_data_, so a longer chunk at a
        // given offset is a strict SUPERSET of a shorter one: keeping the LONGEST is
        // always byte-correct and can only ever retain MORE data. Receiver byte
        // coverage must be MONOTONE — it may grow, never shrink.
        auto staged_it = rx_prestart_chunks_.find(offset);
        const size_t have_len =
            (staged_it == rx_prestart_chunks_.end()) ? 0 : staged_it->second.size();
        if (data_len <= have_len) {
            // Duplicate or SHORTER re-grid of a span we already hold in full — the
            // existing entry already covers these bytes. Drop it rather than shrink.
            if (!more_data) rx_prestart_final_seen_ = true;
            return true;
        }
        // Account only the GROWTH against the staging budget (the shared prefix is
        // already counted in `staged`).
        if (staged + (data_len - have_len) > MAX_PRESTART_STAGE_BYTES) {
            rx_prestart_overflow_ = true;
            LOG_MODEM(WARN,
                      "FileTransfer: pre-FILE_START staging overflow (%zu B staged); "
                      "falling back to NACK-until-FILE_START",
                      staged);
            return true;
        }
        rx_prestart_chunks_[offset] = Bytes(data, data + data_len);
        if (!more_data) rx_prestart_final_seen_ = true;
        LOG_MODEM(INFO,
                  "FileTransfer: STAGED pre-FILE_START chunk offset=%u len=%zu "
                  "(%zu staged) — will drain on FILE_START",
                  offset, data_len, rx_prestart_chunks_.size());
        return true;
    }

    if (!more_data) {
        rx_final_chunk_seen_ = true;
    }

    // With Selective Repeat ARQ (window > 1), chunks can arrive out of order.
    // Buffer out-of-order chunks and insert them when the gap is filled.
    uint32_t expected_offset = static_cast<uint32_t>(rx_data_.size());
    if (offset < expected_offset) {
        if (static_cast<size_t>(offset) + data_len <= expected_offset) {
            LOG_MODEM(DEBUG,
                      "FileTransfer: Ignoring duplicate/overlap chunk offset=%u len=%zu (have=%zu)",
                      offset, data_len, rx_data_.size());
            return true;
        }
        // Straddles the contiguous edge: a sender requeue (ARQ abort on a rate/mod
        // move) resends on the NEW chunk grid, so a resent chunk can start below
        // the edge and extend past it. Bytes at equal offsets are identical
        // (absolute offsets into the same transmitted stream) — append only the
        // unseen tail; dropping the whole chunk would lose those bytes forever.
        const size_t skip = expected_offset - offset;
        rx_data_.insert(rx_data_.end(), data + skip, data + data_len);
        LOG_MODEM(INFO,
                  "FileTransfer: Merged straddling chunk offset=%u len=%zu tail=%zu (total=%zu)",
                  offset, data_len, data_len - skip, rx_data_.size());
    } else if (offset > expected_offset) {
        // Out-of-order: buffer for later insertion. The bytes ARE received and kept
        // (SR-ARQ), they just can't be written until the earlier gap fills. Notify so
        // the bar reflects the true received total now, not only when the gap closes.
        // NEVER-SHRINK coverage — same invariant as the pre-FILE_START staging map
        // above (F181 root cause): a mid-stream rate/mod move re-grids the SAME byte
        // offset to a SHORTER length, and a bare `map[offset] = ...` destroyed the
        // longer copy's tail. Offsets are absolute into one immutable tx_data_, so the
        // longest entry at an offset is a superset of every shorter one. Keep it.
        auto pend_it = rx_pending_chunks_.find(offset);
        if (pend_it != rx_pending_chunks_.end() && pend_it->second.size() >= data_len) {
            LOG_MODEM(INFO,
                      "FileTransfer: out-of-order chunk offset=%u len=%zu already covered "
                      "by a %zu B entry — keeping the longer copy",
                      offset, data_len, pend_it->second.size());
            notifyProgress();
            return true;
        }
        rx_pending_chunks_[offset] = Bytes(data, data + data_len);
        LOG_MODEM(INFO,
                  "FileTransfer: Buffered out-of-order chunk offset=%u len=%zu (expected=%u, pending=%zu)",
                  offset, data_len, expected_offset, rx_pending_chunks_.size());
        notifyProgress();
        return true;
    } else {
        // In-order: append directly
        rx_data_.insert(rx_data_.end(), data, data + data_len);
    }

    // Drain any buffered chunks that are now in sequence (overlap-aware)
    drainPendingChunks();

    notifyProgress();

    // For uncompressed files, trust explicit expected size more than MORE_FRAG.
    if (((rx_flags_ & FileFlags::COMPRESSED) == 0) && !more_data &&
        rx_data_.size() < rx_expected_size_) {
        LOG_MODEM(WARN,
                  "FileTransfer: Premature final chunk ignored (%zu/%u bytes)",
                  rx_data_.size(), rx_expected_size_);
    }

    checkAndFinalizeReceive();
    return true;
}

// Finalize the receive once the data is complete: decompress (if needed), verify CRC,
// write the file, fire the callback. Shared by the normal in-order path AND the
// FILE_START staging drain (a tiny file whose final chunk was staged before FILE_START
// completes on the drain, not on a later data frame). Completion is decided by byte
// count / final-chunk flag, NOT by frame index or descriptor, so it is correct
// regardless of how the chunks were carried over the air.
void FileTransferController::checkAndFinalizeReceive() {
    const bool compressed = (rx_flags_ & FileFlags::COMPRESSED) != 0;
    const bool size_reached = !compressed && rx_data_.size() >= rx_expected_size_;
    const bool compressed_complete =
        compressed && rx_final_chunk_seen_ && rx_pending_chunks_.empty();

    // Finalization trigger:
    // - Compressed: still rely on MORE_FRAG (we don't know compressed size a priori)
    // - Uncompressed: finalize once expected byte count is reached
    if (compressed_complete || (!compressed && size_reached)) {
        Bytes final_data;

        // Decompress if needed
        if (compressed) {
            auto decompressed = Compression::decompress(rx_data_, rx_expected_size_ * 2);
            if (!decompressed) {
                state_ = FileTransferState::ERROR;
                LOG_MODEM(ERROR, "FileTransfer: Decompression failed (%zu bytes compressed)",
                          rx_data_.size());
                if (on_received_) {
                    on_received_("", false, "Decompression failed");
                }
                resetRxState();
                return;
            }
            final_data = std::move(*decompressed);
        } else {
            if (rx_data_.size() > rx_expected_size_) {
                LOG_MODEM(WARN,
                          "FileTransfer: Truncating oversized payload (%zu -> %u bytes)",
                          rx_data_.size(), rx_expected_size_);
                rx_data_.resize(rx_expected_size_);
            }
            final_data = std::move(rx_data_);
        }

        // Verify CRC32
        uint32_t crc = 0xFFFFFFFF;
        crc = updateCRC32(crc, final_data.data(), final_data.size());
        crc ^= 0xFFFFFFFF;

        bool success = (crc == rx_expected_crc_);

        if (success) {
            // Write to file
            std::ofstream out(rx_filepath_, std::ios::binary);
            if (out.good()) {
                out.write(reinterpret_cast<const char*>(final_data.data()), final_data.size());
                out.close();
                LOG_MODEM(INFO, "FileTransfer: Received OK (%zu bytes, crc=%08X) -> %s",
                          final_data.size(), crc, rx_filepath_.c_str());
            } else {
                success = false;
                LOG_MODEM(ERROR, "FileTransfer: Failed to open output file: %s",
                          rx_filepath_.c_str());
            }
        } else {
            LOG_MODEM(ERROR,
                      "FileTransfer: CRC mismatch (got=%08X expected=%08X, size=%zu/%u)",
                      crc, rx_expected_crc_, final_data.size(), rx_expected_size_);
        }

        state_ = success ? FileTransferState::COMPLETE : FileTransferState::ERROR;

        if (on_received_) {
            on_received_(success ? rx_filepath_ : "", success,
                         success ? "" : "CRC mismatch or file write failed");
        }

        resetRxState();
    }
}

bool FileTransferController::processFileBlock(const Bytes& payload) {
    if (payload.size() < FILE_BLOCK_FIXED_OVERHEAD) {
        return true;
    }

    const uint8_t flags = payload[1];
    const uint32_t original_size = (static_cast<uint32_t>(payload[2]) << 24) |
                                   (static_cast<uint32_t>(payload[3]) << 16) |
                                   (static_cast<uint32_t>(payload[4]) << 8) |
                                   static_cast<uint32_t>(payload[5]);
    const uint32_t transmitted_size = (static_cast<uint32_t>(payload[6]) << 24) |
                                      (static_cast<uint32_t>(payload[7]) << 16) |
                                      (static_cast<uint32_t>(payload[8]) << 8) |
                                      static_cast<uint32_t>(payload[9]);
    const uint32_t expected_crc = (static_cast<uint32_t>(payload[10]) << 24) |
                                  (static_cast<uint32_t>(payload[11]) << 16) |
                                  (static_cast<uint32_t>(payload[12]) << 8) |
                                  static_cast<uint32_t>(payload[13]);
    const size_t name_len = payload[14];
    const size_t data_offset = FILE_BLOCK_FIXED_OVERHEAD + name_len;
    if (data_offset > payload.size() || payload.size() - data_offset != transmitted_size) {
        LOG_MODEM(ERROR, "FileTransfer: Malformed FILE_BLOCK (name=%zu, tx=%u, payload=%zu)",
                  name_len, transmitted_size, payload.size());
        return true;
    }

    const std::string filename(payload.begin() + FILE_BLOCK_FIXED_OVERHEAD,
                               payload.begin() + FILE_BLOCK_FIXED_OVERHEAD + name_len);
    const std::string clean_filename = sanitizeReceivedFilename(filename);
    const std::string filepath = uniqueReceivePath(rx_dir_, clean_filename);
    Bytes data(payload.begin() + data_offset, payload.end());

    Bytes final_data;
    if ((flags & FileFlags::COMPRESSED) != 0) {
        auto decompressed = Compression::decompress(data, original_size * 2);
        if (!decompressed) {
            LOG_MODEM(ERROR, "FileTransfer: FILE_BLOCK decompression failed (%zu bytes)",
                      data.size());
            if (on_received_) {
                on_received_("", false, "Decompression failed");
            }
            return true;
        }
        final_data = std::move(*decompressed);
    } else {
        final_data = std::move(data);
    }

    if (final_data.size() != original_size) {
        LOG_MODEM(ERROR, "FileTransfer: FILE_BLOCK size mismatch (%zu/%u bytes)",
                  final_data.size(), original_size);
        if (on_received_) {
            on_received_("", false, "Size mismatch");
        }
        return true;
    }

    uint32_t crc = 0xFFFFFFFF;
    crc = updateCRC32(crc, final_data.data(), final_data.size());
    crc ^= 0xFFFFFFFF;
    bool success = (crc == expected_crc);

    if (success) {
        std::ofstream out(filepath, std::ios::binary);
        if (out.good()) {
            out.write(reinterpret_cast<const char*>(final_data.data()), final_data.size());
            out.close();
            LOG_MODEM(INFO, "FileTransfer: Received block OK (%zu bytes, crc=%08X) -> %s",
                      final_data.size(), crc, filepath.c_str());
        } else {
            success = false;
            LOG_MODEM(ERROR, "FileTransfer: Failed to open block output file: %s",
                      filepath.c_str());
        }
    } else {
        LOG_MODEM(ERROR, "FileTransfer: FILE_BLOCK CRC mismatch (got=%08X expected=%08X)",
                  crc, expected_crc);
    }

    state_ = success ? FileTransferState::COMPLETE : FileTransferState::ERROR;
    if (on_received_) {
        on_received_(success ? filepath : "", success,
                     success ? "" : "CRC mismatch or file write failed");
    }
    resetRxState();
    return true;
}

uint32_t FileTransferController::calculateCRC32(std::istream& stream) {
    uint32_t crc = 0xFFFFFFFF;
    char buffer[4096];

    while (stream.read(buffer, sizeof(buffer)) || stream.gcount() > 0) {
        crc = updateCRC32(crc, reinterpret_cast<uint8_t*>(buffer), stream.gcount());
    }

    return crc ^ 0xFFFFFFFF;
}

uint32_t FileTransferController::calculateCRC32File(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.good()) {
        return 0;
    }
    return calculateCRC32(file);
}

void FileTransferController::resetTxState() {
    tx_delivered_ranges_.clear();  // F163 FIX-4: per-file
    tx_filepath_.clear();
    tx_filename_.clear();
    tx_data_.clear();
    tx_original_size_ = 0;
    tx_crc_ = 0;
    tx_offset_ = 0;
    tx_flags_ = 0;
    tx_metadata_sent_ = false;
    chunks_sent_ = 0;
    chunks_acked_ = 0;
    tx_pending_ledger_.clear();
    state_ = FileTransferState::IDLE;
}

void FileTransferController::resetRxState() {
    rx_filepath_.clear();
    rx_filename_.clear();
    rx_data_.clear();
    rx_pending_chunks_.clear();
    rx_prestart_chunks_.clear();
    rx_prestart_final_seen_ = false;
    rx_prestart_overflow_ = false;
    rx_expected_size_ = 0;
    rx_expected_crc_ = 0;
    rx_flags_ = 0;
    rx_final_chunk_seen_ = false;
    state_ = FileTransferState::IDLE;
}

void FileTransferController::notifyProgress() {
    if (on_progress_) {
        on_progress_(getProgress());
    }
}

std::string FileTransferController::extractFilename(const std::string& path) {
    size_t last_sep = path.find_last_of("/\\");
    if (last_sep != std::string::npos) {
        return path.substr(last_sep + 1);
    }
    return path;
}

} // namespace protocol
} // namespace ultra
