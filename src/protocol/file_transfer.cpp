#include "file_transfer.hpp"
#include "compression.hpp"
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cstring>

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
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7A9B, 0x5005713C, 0x270241AA,
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
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD706B3,
    0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

uint32_t updateCRC32(uint32_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        crc = CRC32_TABLE[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc;
}

} // anonymous namespace

FileTransferController::~FileTransferController() {
    cancel();
}

bool FileTransferController::startSend(const std::string& filepath) {
    if (isBusy()) {
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
    tx_waiting_ack_ = false;
    state_ = FileTransferState::SENDING;

    notifyProgress();
    return true;
}

Bytes FileTransferController::getNextChunk() {
    if (state_ != FileTransferState::SENDING || tx_waiting_ack_) {
        return {};
    }

    Bytes payload;

    if (!tx_metadata_sent_) {
        payload = buildMetadataPayload();
    } else {
        payload = buildDataPayload();
    }

    if (!payload.empty()) {
        tx_waiting_ack_ = true;
    }

    return payload;
}

bool FileTransferController::hasMoreChunks() const {
    if (state_ != FileTransferState::SENDING) {
        return false;
    }
    return !tx_metadata_sent_ || tx_offset_ < tx_data_.size();
}

void FileTransferController::onChunkAcked() {
    if (state_ != FileTransferState::SENDING) {
        return;
    }

    tx_waiting_ack_ = false;

    if (!tx_metadata_sent_) {
        tx_metadata_sent_ = true;
    }

    notifyProgress();

    if (!hasMoreChunks()) {
        state_ = FileTransferState::COMPLETE;

        if (on_sent_) {
            on_sent_(true, "");
        }

        resetTxState();
    }
}

void FileTransferController::onSendFailed() {
    if (state_ != FileTransferState::SENDING) {
        return;
    }

    state_ = FileTransferState::ERROR;

    if (on_sent_) {
        on_sent_(false, "Transfer failed: max retries exceeded");
    }

    resetTxState();
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
        progress.is_sending = true;
    } else if (state_ == FileTransferState::RECEIVING) {
        progress.filename = rx_filename_;
        progress.total_bytes = rx_expected_size_;
        progress.transferred_bytes = static_cast<uint32_t>(rx_data_.size());
        progress.is_sending = false;
    }

    return progress;
}

bool FileTransferController::isBusy() const {
    return state_ == FileTransferState::SENDING ||
           state_ == FileTransferState::RECEIVING;
}

void FileTransferController::cancel() {
    if (state_ == FileTransferState::SENDING) {
        if (on_sent_) {
            on_sent_(false, "Transfer cancelled");
        }
        resetTxState();
    } else if (state_ == FileTransferState::RECEIVING) {
        if (on_received_) {
            on_received_("", false);
        }
        resetRxState();
    }

    state_ = FileTransferState::IDLE;
}

// --- Private Methods ---

Bytes FileTransferController::buildMetadataPayload() {
    // Format: TYPE(1) + FLAGS(1) + ORIGINAL_SIZE(4) + CRC32(4) + FILENAME(variable)
    Bytes payload;
    payload.reserve(1 + 1 + 4 + 4 + tx_filename_.size());

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

    // Filename (truncate if too long)
    size_t max_name_len = 256 - 10;
    std::string name = tx_filename_.substr(0, max_name_len);
    payload.insert(payload.end(), name.begin(), name.end());

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
    size_t chunk_size = std::min(remaining, CHUNK_SIZE);

    payload.insert(payload.end(),
                   tx_data_.begin() + tx_offset_,
                   tx_data_.begin() + tx_offset_ + chunk_size);

    tx_offset_ += static_cast<uint32_t>(chunk_size);

    return payload;
}

bool FileTransferController::processFileStart(const Bytes& payload) {
    // Format: TYPE(1) + FLAGS(1) + SIZE(4) + CRC32(4) + FILENAME
    if (payload.size() < 11) {
        return true;  // Malformed
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

    // Parse filename
    rx_filename_ = std::string(payload.begin() + 10, payload.end());

    // Sanitize filename
    std::replace(rx_filename_.begin(), rx_filename_.end(), '/', '_');
    std::replace(rx_filename_.begin(), rx_filename_.end(), '\\', '_');
    std::replace(rx_filename_.begin(), rx_filename_.end(), ':', '_');

    // Create output path
    rx_filepath_ = rx_dir_ + "/" + rx_filename_;

    // Handle duplicate filenames
    int suffix = 1;
    std::string base_path = rx_filepath_;
    while (std::filesystem::exists(rx_filepath_)) {
        size_t dot_pos = base_path.rfind('.');
        if (dot_pos != std::string::npos) {
            rx_filepath_ = base_path.substr(0, dot_pos) + "_" +
                          std::to_string(suffix) + base_path.substr(dot_pos);
        } else {
            rx_filepath_ = base_path + "_" + std::to_string(suffix);
        }
        suffix++;
    }

    rx_data_.clear();
    rx_data_.reserve(rx_expected_size_);  // Reserve for decompressed size as estimate
    state_ = FileTransferState::RECEIVING;

    notifyProgress();
    return true;
}

bool FileTransferController::processFileData(const Bytes& payload, bool more_data) {
    if (payload.size() < 6) {
        return true;
    }

    if (state_ != FileTransferState::RECEIVING) {
        return true;
    }

    // Append data (skip type and offset bytes)
    const uint8_t* data = payload.data() + 5;
    size_t data_len = payload.size() - 5;

    rx_data_.insert(rx_data_.end(), data, data + data_len);

    notifyProgress();

    // Check if this is the last chunk
    if (!more_data) {
        Bytes final_data;

        // Decompress if needed
        if (rx_flags_ & FileFlags::COMPRESSED) {
            auto decompressed = Compression::decompress(rx_data_, rx_expected_size_ * 2);
            if (!decompressed) {
                state_ = FileTransferState::ERROR;
                if (on_received_) {
                    on_received_("", false);
                }
                resetRxState();
                return true;
            }
            final_data = std::move(*decompressed);
        } else {
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
            } else {
                success = false;
            }
        }

        state_ = success ? FileTransferState::COMPLETE : FileTransferState::ERROR;

        if (on_received_) {
            on_received_(success ? rx_filepath_ : "", success);
        }

        resetRxState();
    }

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
    tx_filepath_.clear();
    tx_filename_.clear();
    tx_data_.clear();
    tx_original_size_ = 0;
    tx_crc_ = 0;
    tx_offset_ = 0;
    tx_flags_ = 0;
    tx_metadata_sent_ = false;
    tx_waiting_ack_ = false;
    state_ = FileTransferState::IDLE;
}

void FileTransferController::resetRxState() {
    rx_filepath_.clear();
    rx_filename_.clear();
    rx_data_.clear();
    rx_expected_size_ = 0;
    rx_expected_crc_ = 0;
    rx_flags_ = 0;
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
