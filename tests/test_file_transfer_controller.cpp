#include "protocol/file_transfer.hpp"
#include "helpers/temp_dir.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using namespace ultra;
using namespace ultra::protocol;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
            return; \
        } \
    } while (0)

struct TxChunk {
    Bytes payload;
    bool more_data = false;
};

Bytes makeCompressiblePayload() {
    Bytes data;
    const std::string phrase =
        "ProjectUltra HF modem file-transfer compressed reorder regression.\n";
    for (int i = 0; i < 512; ++i) {
        data.insert(data.end(), phrase.begin(), phrase.end());
        data.push_back(static_cast<uint8_t>('A' + (i % 4)));
    }
    return data;
}

bool writeFile(const std::filesystem::path& path, const Bytes& data) {
    std::ofstream out(path, std::ios::binary);
    if (!out.good()) {
        return false;
    }
    out.write(reinterpret_cast<const char*>(data.data()), data.size());
    return out.good();
}

Bytes readFile(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in.good()) {
        return {};
    }

    const std::streamsize size = in.tellg();
    in.seekg(0, std::ios::beg);

    Bytes data(static_cast<size_t>(size));
    in.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

std::vector<TxChunk> collectChunks(FileTransferController& tx) {
    std::vector<TxChunk> chunks;
    while (tx.hasMoreChunks()) {
        Bytes payload = tx.getNextChunk();
        if (!payload.empty()) {
            chunks.push_back({std::move(payload), tx.hasMoreChunks()});
        }
    }
    return chunks;
}

struct TransferDirs {
    ultra::test::TempDir root;
    std::filesystem::path tx_dir;
    std::filesystem::path rx_dir;
    bool ready = false;

    explicit TransferDirs(const std::string& prefix, const std::string& rx_name = "rx")
        : root(prefix),
          tx_dir(root.child("tx")),
          rx_dir(root.child(rx_name)) {
        if (!root.valid()) {
            return;
        }
        std::error_code ec;
        std::filesystem::create_directories(tx_dir, ec);
        if (ec) {
            return;
        }
        std::filesystem::create_directories(rx_dir, ec);
        ready = !ec;
    }
};

void test_failure_callback_can_start_next_file_without_outer_reset_wiping_it() {
    TransferDirs dirs("ultra_file_transfer_reentrant_failure_test");
    CHECK(dirs.ready, "create reentrant failure temp directories");

    const std::filesystem::path first_path = dirs.tx_dir / "first.bin";
    const std::filesystem::path second_path = dirs.tx_dir / "second.bin";
    CHECK(writeFile(first_path, Bytes{1, 2, 3, 4}), "write first source file");
    CHECK(writeFile(second_path, Bytes{5, 6, 7, 8, 9}), "write second source file");

    FileTransferController tx;
    CHECK(tx.startSend(first_path.string()), "start first transfer");
    bool callback_called = false;
    bool restarted = false;
    tx.setSentCallback([&](bool success, const std::string&) {
        callback_called = true;
        CHECK(!success, "terminal callback should report failure");
        restarted = tx.startSend(second_path.string());
    });

    tx.onSendFailed();
    CHECK(callback_called && restarted,
          "failure callback should synchronously start the replacement file");
    CHECK(tx.getState() == FileTransferState::SENDING,
          "outer failure cleanup must not wipe the replacement transfer state");
    CHECK(tx.getProgress().filename == second_path.filename().string(),
          "replacement file metadata must survive callback return");
    Bytes metadata = tx.getNextChunk();
    CHECK(!metadata.empty() &&
              metadata[0] == static_cast<uint8_t>(PayloadType::FILE_START),
          "replacement transfer should retain a valid FILE_START chunk");

    tx.setSentCallback({});
    tx.cancel();
}

void test_success_callback_can_start_next_file_without_outer_reset_wiping_it() {
    TransferDirs dirs("ultra_file_transfer_reentrant_success_test");
    CHECK(dirs.ready, "create reentrant success temp directories");

    const std::filesystem::path first_path = dirs.tx_dir / "first-success.bin";
    const std::filesystem::path second_path = dirs.tx_dir / "second-success.bin";
    CHECK(writeFile(first_path, Bytes{1, 2, 3, 4}), "write first success source file");
    CHECK(writeFile(second_path, Bytes{5, 6, 7, 8, 9}),
          "write second success source file");

    FileTransferController tx;
    CHECK(tx.startSend(first_path.string()), "start first successful transfer");
    const std::vector<TxChunk> first_chunks = collectChunks(tx);
    CHECK(!first_chunks.empty(), "first transfer should produce chunks to retire");

    bool callback_called = false;
    bool callback_success = false;
    bool restarted = false;
    tx.setSentCallback([&](bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        restarted = tx.startSend(second_path.string());
    });
    for (size_t i = 0; i < first_chunks.size(); ++i) {
        tx.onChunkAcked();
    }

    CHECK(callback_called && callback_success && restarted,
          "success callback should synchronously start the replacement file");
    CHECK(tx.getState() == FileTransferState::SENDING,
          "outer success cleanup must not wipe the replacement transfer state");
    CHECK(tx.getProgress().filename == second_path.filename().string(),
          "replacement file metadata must survive success callback return");
    Bytes metadata = tx.getNextChunk();
    CHECK(!metadata.empty() &&
              metadata[0] == static_cast<uint8_t>(PayloadType::FILE_START),
          "replacement transfer should retain a valid FILE_START chunk");

    tx.setSentCallback({});
    tx.cancel();
}

void test_compressed_final_chunk_out_of_order_finalizes() {
    TransferDirs dirs("ultra_file_transfer_controller_test");
    CHECK(dirs.ready, "create temp directories");

    const Bytes original = makeCompressiblePayload();
    const std::filesystem::path src_path = dirs.tx_dir / "compressed_reorder.txt";
    CHECK(writeFile(src_path, original), "write source file");

    FileTransferController tx;
    tx.setMaxChunkPayload(37);  // 32 compressed data bytes per FILE_DATA chunk.
    CHECK(tx.startSend(src_path.string()), "start compressed send");

    std::vector<TxChunk> chunks = collectChunks(tx);
    CHECK(chunks.size() >= 4, "compressed transfer should span metadata plus multiple data chunks");
    CHECK(chunks.front().payload[0] == static_cast<uint8_t>(PayloadType::FILE_START),
          "first chunk is metadata");
    CHECK(chunks.back().payload[0] == static_cast<uint8_t>(PayloadType::FILE_DATA),
          "last chunk is file data");
    CHECK(!chunks.back().more_data, "last chunk carries final marker");

    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());

    bool callback_called = false;
    bool callback_success = false;
    std::string received_path;
    rx.setReceivedCallback([&](const std::string& path, bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        received_path = path;
    });

    CHECK(rx.processPayload(chunks.front().payload, chunks.front().more_data),
          "metadata accepted");

    const size_t last = chunks.size() - 1;
    CHECK(rx.processPayload(chunks[last].payload, chunks[last].more_data),
          "out-of-order final chunk accepted");
    CHECK(!callback_called, "out-of-order final chunk should not finalize before gaps close");

    for (size_t i = 1; i < last; ++i) {
        CHECK(rx.processPayload(chunks[i].payload, chunks[i].more_data),
              "gap-filling chunk accepted");
    }

    CHECK(callback_called, "receiver callback fired after final buffered chunk drained");
    CHECK(callback_success, "receiver reported success");
    CHECK(readFile(received_path) == original, "received file matches original payload");
}

void test_duplicate_filename_in_dotted_receive_directory() {
    TransferDirs dirs("ultra_file_transfer_duplicate_test", "rx.with.dot");
    CHECK(dirs.ready, "create temp directories");

    const Bytes original = {'P', 'r', 'o', 'j', 'e', 'c', 't',
                            'U', 'l', 't', 'r', 'a'};
    const std::filesystem::path src_path = dirs.tx_dir / "payload";
    CHECK(writeFile(src_path, original), "write duplicate source file");
    CHECK(writeFile(dirs.rx_dir / "payload", Bytes{'o', 'l', 'd'}),
          "write existing receive file");

    FileTransferController tx;
    CHECK(tx.startSend(src_path.string()), "start duplicate-name send");
    std::vector<TxChunk> chunks = collectChunks(tx);
    CHECK(chunks.size() >= 2, "duplicate-name transfer has metadata and data");

    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());

    bool callback_called = false;
    bool callback_success = false;
    std::string received_path;
    rx.setReceivedCallback([&](const std::string& path, bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        received_path = path;
    });

    for (const auto& chunk : chunks) {
        CHECK(rx.processPayload(chunk.payload, chunk.more_data),
              "duplicate-name chunk accepted");
    }

    const std::filesystem::path received(received_path);
    CHECK(callback_called, "duplicate-name receiver callback fired");
    CHECK(callback_success, "duplicate-name transfer succeeded");
    CHECK(received.parent_path() == dirs.rx_dir,
          "duplicate-name uniquing stays inside dotted receive directory");
    CHECK(received.filename() == "payload_1",
          "duplicate-name uniquing appends suffix to filename without extension");
    CHECK(readFile(received) == original, "duplicate-name received content matches");
    CHECK(readFile(dirs.rx_dir / "payload") == Bytes({'o', 'l', 'd'}),
          "duplicate-name original file preserved");
}

void test_single_block_payload_round_trip() {
    TransferDirs dirs("ultra_file_transfer_block_test");
    CHECK(dirs.ready, "create temp directories");

    const Bytes original = {'b', 'l', 'o', 'c', 'k', 0, 1, 2, 3, 4};
    const std::filesystem::path src_path = dirs.tx_dir / "block.bin";
    CHECK(writeFile(src_path, original), "write block source file");

    FileTransferController tx;
    CHECK(tx.startSend(src_path.string()), "start block send");
    Bytes block = tx.getSingleBlockPayload(1024);
    CHECK(!block.empty(), "single block payload fits");
    CHECK(block[0] == static_cast<uint8_t>(PayloadType::FILE_BLOCK),
          "single block uses FILE_BLOCK type");
    CHECK(!tx.hasMoreChunks(), "single block consumes tx chunks");

    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());
    bool callback_called = false;
    bool callback_success = false;
    std::string received_path;
    rx.setReceivedCallback([&](const std::string& path, bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        received_path = path;
    });

    CHECK(rx.processPayload(block, false), "block payload accepted");
    CHECK(callback_called, "block receiver callback fired");
    CHECK(callback_success, "block transfer succeeded");
    CHECK(readFile(received_path) == original, "block received content matches");
}

void test_file_start_requires_eleven_bytes_but_later_data_may_use_eight() {
    TransferDirs dirs("ultra_file_transfer_min_metadata_test");
    CHECK(dirs.ready, "create minimum-metadata temp directories");

    const Bytes original = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87, 0x98};
    const std::filesystem::path src_path = dirs.tx_dir / "minimum.bin";
    CHECK(writeFile(src_path, original), "write minimum-metadata source file");

    FileTransferController too_small_8;
    too_small_8.setMaxChunkPayload(8);
    CHECK(!too_small_8.startSend(src_path.string()),
          "8-byte payload can carry FILE_DATA but must reject a new FILE_START");
    CHECK(too_small_8.getState() == FileTransferState::IDLE &&
              too_small_8.getNextChunk().empty(),
          "rejected 8-byte start must remain idle and emit no malformed metadata");

    FileTransferController too_small_10;
    too_small_10.setMaxChunkPayload(10);
    CHECK(!too_small_10.startSend(src_path.string()),
          "10-byte payload without a filename byte must reject FILE_START");
    CHECK(too_small_10.getState() == FileTransferState::IDLE &&
              too_small_10.getNextChunk().empty(),
          "rejected 10-byte start must remain idle and emit no nameless metadata");

    FileTransferController tx;
    tx.setMaxChunkPayload(FileTransferController::MIN_FILE_START_PAYLOAD);
    CHECK(tx.startSend(src_path.string()), "11-byte payload is the minimum valid file start");
    const Bytes metadata = tx.getNextChunk();
    CHECK(metadata.size() == FileTransferController::MIN_FILE_START_PAYLOAD &&
              metadata[0] == static_cast<uint8_t>(PayloadType::FILE_START),
          "minimum FILE_START must fill exactly 11 bytes");
    CHECK(metadata[10] == static_cast<uint8_t>(src_path.filename().string().front()),
          "minimum FILE_START must retain one filename byte");

    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());
    bool callback_called = false;
    bool callback_success = false;
    std::string received_path;
    rx.setReceivedCallback([&](const std::string& path, bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        received_path = path;
    });
    CHECK(rx.processPayload(metadata, true), "minimum FILE_START is accepted by receiver");

    // Once metadata is established, an 8-byte geometry remains useful: its
    // 5-byte FILE_DATA header leaves three file bytes per frame.
    tx.setMaxChunkPayload(8);
    size_t data_frames = 0;
    while (tx.hasMoreChunks()) {
        const Bytes data = tx.getNextChunk();
        CHECK(!data.empty() && data.size() >= 6 && data.size() <= 8 &&
                  data[0] == static_cast<uint8_t>(PayloadType::FILE_DATA),
              "post-metadata 8-byte geometry must emit bounded non-empty FILE_DATA");
        const bool more_data = tx.hasMoreChunks();
        CHECK(rx.processPayload(data, more_data), "8-byte FILE_DATA is accepted");
        ++data_frames;
    }

    CHECK(data_frames == 3, "nine bytes should use three 3-byte FILE_DATA frames");
    CHECK(callback_called && callback_success,
          "minimum metadata plus later 8-byte FILE_DATA must complete");
    CHECK(readFile(received_path) == original,
          "minimum metadata plus small DATA geometry must preserve exact bytes");
}

uint32_t dataPayloadOffset(const Bytes& payload) {
    // FILE_DATA: TYPE(1) + TRANSFER ID(1) + OFFSET(3, big-endian) + DATA
    return fileTransferValueOf((static_cast<uint32_t>(payload[1]) << 24) |
                               (static_cast<uint32_t>(payload[2]) << 16) |
                               (static_cast<uint32_t>(payload[3]) << 8) |
                               static_cast<uint32_t>(payload[4]));
}

Bytes makeIncompressiblePayload(size_t size) {
    // LCG noise so deflate can't shrink it — transmitted bytes == file bytes,
    // keeping chunk offsets deterministic in transmitted space.
    Bytes data(size);
    uint32_t x = 0x2545F491u;
    for (size_t i = 0; i < size; ++i) {
        x = x * 1664525u + 1013904223u;
        data[i] = static_cast<uint8_t>(x >> 24);
    }
    return data;
}

// Regression for the Moderate@20 ladder data-loss bug (2026-07-02): the requeue
// resume offset was reconstructed as (chunks_acked_-1)*chunk_size_, which is
// wrong the moment chunk_size_ has changed mid-file (mid-stream rate/mod move) —
// it jumped the cursor FORWARD 10 KB past unsent bytes and the sender declared
// completion while the receiver held a permanent in-order hole. The ledger fix
// must resume exactly at the oldest un-retired chunk's offset.
void test_requeue_resumes_exact_offset_across_chunk_size_changes() {
    TransferDirs dirs("ultra_file_transfer_requeue_test");
    CHECK(dirs.ready, "create temp directories");

    const Bytes original = makeIncompressiblePayload(1000);
    const std::filesystem::path src_path = dirs.tx_dir / "requeue.bin";
    CHECK(writeFile(src_path, original), "write requeue source file");

    FileTransferController tx;
    tx.setMaxChunkPayload(25);  // 20 data bytes per chunk (era 1)
    CHECK(tx.startSend(src_path.string()), "start requeue send");

    // Metadata out + retired.
    Bytes meta = tx.getNextChunk();
    CHECK(!meta.empty() && meta[0] == static_cast<uint8_t>(PayloadType::FILE_START),
          "first chunk is metadata");
    tx.onChunkAcked();

    // Era 1: three 20-byte chunks at offsets 0/20/40; retire the first two.
    Bytes c0 = tx.getNextChunk();
    Bytes c1 = tx.getNextChunk();
    Bytes c2 = tx.getNextChunk();
    CHECK(dataPayloadOffset(c0) == 0 && dataPayloadOffset(c1) == 20 &&
              dataPayloadOffset(c2) == 40,
          "era-1 chunk offsets are 0/20/40");
    tx.onChunkAcked();  // offset 0 retired
    tx.onChunkAcked();  // offset 20 retired; oldest un-retired is now offset 40

    // Mid-stream move: chunk size changes 20 -> 40 (heterogeneous history).
    tx.setMaxChunkPayload(45);
    Bytes c3 = tx.getNextChunk();
    Bytes c4 = tx.getNextChunk();
    CHECK(dataPayloadOffset(c3) == 60 && dataPayloadOffset(c4) == 100,
          "era-2 chunk offsets are 60/100");

    // ARQ abort (code-rate change with a busy window) -> requeue. The old
    // count-based arithmetic would resume at (3-1)*40 = 80, silently skipping
    // bytes [40,80). The ledger must resume at 40.
    tx.requeuePendingChunks();
    Bytes resumed = tx.getNextChunk();
    CHECK(!resumed.empty() && resumed[0] == static_cast<uint8_t>(PayloadType::FILE_DATA),
          "resumed chunk is file data");
    CHECK(dataPayloadOffset(resumed) == 40,
          "requeue resumes at the oldest un-retired offset (40), not count*size");

    // Metadata-in-flight case: a requeue with the metadata chunk un-retired
    // must rebuild from scratch (metadata first, then offset 0).
    FileTransferController tx2;
    tx2.setMaxChunkPayload(25);
    CHECK(tx2.startSend(src_path.string()), "start metadata-requeue send");
    Bytes meta2 = tx2.getNextChunk();
    Bytes d0 = tx2.getNextChunk();
    CHECK(!meta2.empty() && !d0.empty(), "metadata + first data chunk pulled");
    tx2.requeuePendingChunks();  // nothing retired: metadata is the oldest pending
    Bytes meta_again = tx2.getNextChunk();
    CHECK(!meta_again.empty() &&
              meta_again[0] == static_cast<uint8_t>(PayloadType::FILE_START),
          "requeue with un-retired metadata rebuilds from the metadata chunk");
}

Bytes makeFileDataPayload(const Bytes& src, uint32_t offset, size_t len, uint8_t transfer_id = 0) {
    // FILE_DATA: TYPE(1) + TRANSFER ID(1) + OFFSET(3, big-endian) + DATA (id 0: no identity)
    Bytes p;
    p.push_back(static_cast<uint8_t>(PayloadType::FILE_DATA));
    p.push_back(transfer_id != 0 ? transfer_id : static_cast<uint8_t>((offset >> 24) & 0xFF));
    p.push_back(static_cast<uint8_t>((offset >> 16) & 0xFF));
    p.push_back(static_cast<uint8_t>((offset >> 8) & 0xFF));
    p.push_back(static_cast<uint8_t>(offset & 0xFF));
    p.insert(p.end(), src.begin() + offset, src.begin() + offset + len);
    return p;
}

// Regression for the receiver half of the requeue story (review findings, 2026-07-02):
// after a sender requeue resends on a CHANGED chunk grid, a resent chunk can start
// below the receiver's contiguous edge and extend past it. The old code dropped any
// offset < expected chunk in its entirety (losing the unseen tail forever) and the
// old drain matched exact offsets only (stranding covered buffered entries, which
// blocks compressed finalization). The receiver must tail-merge straddlers and
// drop/tail-drain buffered entries the edge has overtaken.
void test_receiver_merges_straddling_resend_and_drains_covered_buffered() {
    TransferDirs dirs("ultra_file_transfer_straddle_test");
    CHECK(dirs.ready, "create temp directories");

    const Bytes original = makeIncompressiblePayload(80);  // incompressible -> uncompressed path
    const std::filesystem::path src_path = dirs.tx_dir / "straddle.bin";
    CHECK(writeFile(src_path, original), "write straddle source file");

    FileTransferController tx;
    tx.setMaxChunkPayload(25);  // 20-byte chunks: [0,20) [20,40) [40,60) [60,80)
    CHECK(tx.startSend(src_path.string()), "start straddle send");
    Bytes meta = tx.getNextChunk();
    Bytes c0 = tx.getNextChunk();
    Bytes c1 = tx.getNextChunk();
    Bytes c2 = tx.getNextChunk();
    Bytes c3 = tx.getNextChunk();
    (void)c1;
    CHECK(!tx.hasMoreChunks(), "four data chunks cover the file");

    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());
    bool callback_called = false;
    bool callback_success = false;
    std::string received_path;
    rx.setReceivedCallback([&](const std::string& path, bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        received_path = path;
    });

    CHECK(rx.processPayload(meta, true), "metadata accepted");
    CHECK(rx.processPayload(c0, true), "chunk [0,20) accepted");        // edge = 20
    CHECK(rx.processPayload(c2, true), "chunk [40,60) buffered");       // out-of-order
    // Requeue resend on a different grid: [10,50) straddles the edge (20).
    // Tail [20,50) must append, then the buffered [40,60) entry straddles the
    // new edge (50) and must tail-drain to 60 — the old code lost [20,50) and
    // stranded the buffered entry.
    CHECK(rx.processPayload(makeFileDataPayload(original, 10, 40), true),
          "straddling resend accepted");
    // Pure duplicate fully below the edge is still ignored.
    CHECK(rx.processPayload(makeFileDataPayload(original, 0, 20), true),
          "pure duplicate accepted (ignored)");
    CHECK(!callback_called, "not finalized before the last chunk");
    CHECK(rx.processPayload(c3, false), "final chunk [60,80) accepted");

    CHECK(callback_called, "receiver finalized");
    CHECK(callback_success, "receive succeeded (CRC ok)");
    CHECK(readFile(received_path) == original, "received bytes are exact");
}

}  // namespace

// BUG-SACK-DURABILITY-RESIDUAL root cause (F181, 2026-07-07; fixed 2026-07-24).
// Receiver byte coverage MUST be MONOTONE. chunk_size_ changes on every mid-stream
// rate/mod move, so the SAME byte offset legitimately re-arrives with a SHORTER length
// on the new grid. Both receive maps used a bare `map[offset] = Bytes(...)`, which
// REPLACED a longer buffered/staged copy with the shorter re-grid and DESTROYED the
// tail bytes. In F181 a 456 B chunk at offset 0 was overwritten by era-1's re-gridded
// 408 B chunk, so [408,456) ceased to exist; with the sender-side range salvage on,
// those bytes were then never re-sent => permanently stranded file (900 s, 0 progress).
// Offsets are absolute into one immutable source, so the longest entry at an offset is
// a strict superset of every shorter one: keep it.
void test_receiver_coverage_never_shrinks_on_regrid_out_of_order() {
    TransferDirs dirs("ultra_file_transfer_regrid_ooo_test");
    CHECK(dirs.ready, "create temp directories");

    const Bytes original = makeIncompressiblePayload(80);  // uncompressed path
    const std::filesystem::path src_path = dirs.tx_dir / "regrid_ooo.bin";
    CHECK(writeFile(src_path, original), "write regrid source file");

    FileTransferController tx;
    tx.setMaxChunkPayload(25);  // 20-byte chunks: [0,20) [20,40) [40,60) [60,80)
    CHECK(tx.startSend(src_path.string()), "start regrid send");
    Bytes meta = tx.getNextChunk();

    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());
    bool callback_called = false;
    bool callback_success = false;
    std::string received_path;
    rx.setReceivedCallback([&](const std::string& path, bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        received_path = path;
    });

    CHECK(rx.processPayload(meta, true), "metadata accepted");
    CHECK(rx.processPayload(makeFileDataPayload(original, 0, 20), true),
          "chunk [0,20) accepted");                       // contiguous edge = 20
    CHECK(rx.processPayload(makeFileDataPayload(original, 40, 20), true),
          "chunk [40,60) buffered out-of-order");         // rx_pending_chunks_[40] = 20 B
    // RATE DEMOTE: the sender re-grids to a SHORTER chunk and resends the same offset.
    // The old code overwrote the 20 B entry with 12 B, destroying [52,60).
    CHECK(rx.processPayload(makeFileDataPayload(original, 40, 12), true),
          "shorter re-grid at the same offset accepted");
    // Fill the gap: the edge advances to 40 and must drain the FULL 20 B buffered
    // entry to 60. Under the old code it drained only 12 B (edge 52), leaving the
    // hole [52,60) that nothing ever refills -> the file never finalizes.
    CHECK(rx.processPayload(makeFileDataPayload(original, 20, 20), true),
          "gap chunk [20,40) accepted");
    CHECK(rx.processPayload(makeFileDataPayload(original, 60, 20), false),
          "final chunk [60,80) accepted");

    CHECK(callback_called, "receiver finalized (old code stranded on the [52,60) hole)");
    CHECK(callback_success, "receive succeeded (CRC ok)");
    CHECK(readFile(received_path) == original, "received bytes are exact");
}

// Same invariant on the OTHER map: pre-FILE_START staging (rx_prestart_chunks_).
// This is the exact F181 site — burst data arrives before FILE_START, gets staged by
// byte offset, and a rate demote mid-staging re-grids the same offset shorter.
void test_prestart_staging_coverage_never_shrinks_on_regrid() {
    TransferDirs dirs("ultra_file_transfer_regrid_prestart_test");
    CHECK(dirs.ready, "create temp directories");

    const Bytes original = makeIncompressiblePayload(80);
    const std::filesystem::path src_path = dirs.tx_dir / "regrid_prestart.bin";
    CHECK(writeFile(src_path, original), "write prestart source file");

    FileTransferController tx;
    tx.setMaxChunkPayload(25);
    CHECK(tx.startSend(src_path.string()), "start prestart send");
    Bytes meta = tx.getNextChunk();
    const uint8_t id = meta.size() > 2 ? meta[2] : 0;  // FILE_START: TYPE, FLAGS, ID + SIZE

    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());
    bool callback_called = false;
    bool callback_success = false;
    std::string received_path;
    rx.setReceivedCallback([&](const std::string& path, bool success, const std::string&) {
        callback_called = true;
        callback_success = success;
        received_path = path;
    });

    // Data BEFORE FILE_START -> staged by absolute offset.
    CHECK(rx.processPayload(makeFileDataPayload(original, 0, 20, id), true),
          "staged [0,20) before FILE_START");
    // Rate demote mid-staging: same offset, SHORTER. Must not shrink coverage.
    CHECK(rx.processPayload(makeFileDataPayload(original, 0, 12, id), true),
          "shorter re-grid of a staged offset accepted");
    CHECK(rx.processPayload(makeFileDataPayload(original, 20, 20, id), true),
          "staged [20,40)");
    // FILE_START drains the staging map; the drain must see the FULL 20 B at offset 0.
    CHECK(rx.processPayload(meta, true), "metadata accepted (drains staging)");
    CHECK(rx.processPayload(makeFileDataPayload(original, 40, 20, id), true),
          "chunk [40,60) accepted");
    CHECK(rx.processPayload(makeFileDataPayload(original, 60, 20, id), false),
          "final chunk [60,80) accepted");

    CHECK(callback_called, "receiver finalized (old code stranded on the [12,20) hole)");
    CHECK(callback_success, "receive succeeded (CRC ok)");
    CHECK(readFile(received_path) == original, "received bytes are exact");
}

// BUG-FILE-CHUNK-CROSSES-TRANSFERS (TNC marathon, ITU Good 10 dB, 2026-11-23): an ARQ resend of a
// finished transfer's chunks arrived below the window after it completed; the receiver staged them
// as the NEXT transfer's first bytes, and that 4097-byte block failed its CRC. Every frame now names
// its transfer: late frames of a finished one are dropped, a late FILE_START opens nothing, a late
// FILE_BLOCK is not delivered twice, and the next transfer arrives intact.
void test_late_frames_of_a_finished_transfer_never_reach_the_next() {
    TransferDirs dirs("ultra_file_transfer_identity_test");
    CHECK(dirs.ready, "create temp directories");
    const Bytes first = makeIncompressiblePayload(905);
    Bytes second = makeIncompressiblePayload(4097);
    for (auto& b : second) b = static_cast<uint8_t>(b ^ 0x5A);  // different bytes, same offsets
    const auto first_path = dirs.tx_dir / "first.bin";
    const auto second_path = dirs.tx_dir / "second.bin";
    CHECK(writeFile(first_path, first) && writeFile(second_path, second), "write sources");

    FileTransferController tx;
    tx.setMaxChunkPayload(312);
    FileTransferController rx;
    rx.setReceiveDirectory(dirs.rx_dir.string());
    std::vector<std::pair<bool, std::string>> received;
    rx.setReceivedCallback([&](const std::string& path, bool ok, const std::string&) {
        received.emplace_back(ok, path);
    });

    CHECK(tx.startSend(first_path.string()), "start the first transfer");
    const auto a = collectChunks(tx);
    CHECK(a.size() >= 3, "the first transfer should span several chunks");
    tx.cancel();
    for (const auto& c : a) rx.processPayload(c.payload, c.more_data);
    CHECK(received.size() == 1 && received[0].first, "the first transfer arrives");
    CHECK(readFile(received[0].second) == first, "the first transfer is exact");

    // The late resend: every frame of the finished transfer again, FILE_START included.
    for (const auto& c : a) rx.processPayload(c.payload, c.more_data);
    CHECK(rx.getState() != FileTransferState::RECEIVING,
          "a late FILE_START of a finished transfer must not open a receive");
    CHECK(!rx.hasHealthyStagedData(), "late chunks of a finished transfer must not be staged");
    CHECK(received.size() == 1, "nothing is delivered twice");

    CHECK(tx.startSend(second_path.string()), "start the second transfer");
    const auto b = collectChunks(tx);
    for (const auto& c : b) rx.processPayload(c.payload, c.more_data);
    CHECK(received.size() == 2 && received[1].first, "the second transfer passes its CRC");
    CHECK(readFile(received[1].second) == second, "the second transfer is exact");
    tx.cancel();

    // A single-frame block resent after delivery is not delivered again.
    const auto small_path = dirs.tx_dir / "small.bin";
    CHECK(writeFile(small_path, Bytes{1, 2, 3}), "write a small source");
    CHECK(tx.startSend(small_path.string()), "start a single-block transfer");
    const Bytes block = tx.getSingleBlockPayload(312);
    CHECK(!block.empty(), "a 3-byte file fits one block");
    rx.processPayload(block, false);
    rx.processPayload(block, false);
    CHECK(received.size() == 3, "a resent FILE_BLOCK must not be delivered twice");

    // A new session forgets the identities (a new peer's transfer 1 is not a duplicate).
    rx.beginSession();
    rx.processPayload(block, false);
    CHECK(received.size() == 4, "after a new session the same identity is a new transfer");
}

int main() {
    test_failure_callback_can_start_next_file_without_outer_reset_wiping_it();
    test_success_callback_can_start_next_file_without_outer_reset_wiping_it();
    test_receiver_coverage_never_shrinks_on_regrid_out_of_order();
    test_prestart_staging_coverage_never_shrinks_on_regrid();
    test_compressed_final_chunk_out_of_order_finalizes();
    test_duplicate_filename_in_dotted_receive_directory();
    test_single_block_payload_round_trip();
    test_file_start_requires_eleven_bytes_but_later_data_may_use_eight();
    test_requeue_resumes_exact_offset_across_chunk_size_changes();
    test_receiver_merges_straddling_resend_and_drains_covered_buffered();
    test_late_frames_of_a_finished_transfer_never_reach_the_next();

    if (tests_failed != 0) {
        std::cout << "FileTransferController: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "FileTransferController: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
