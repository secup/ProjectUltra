#pragma once

#include "ultra/types.hpp"
#include "widgets/constellation.hpp"
#include "widgets/controls.hpp"
#include "widgets/status.hpp"
#include "widgets/settings.hpp"
#include "widgets/waterfall.hpp"
#include "widgets/file_browser.hpp"
#include "image_util.hpp"
#include "audio_engine.hpp"
#include "otasim_client/ota_audio_backend.hpp"
#include "modem/modem_engine.hpp"
#include "ptt/ptt_driver.hpp"
#include "protocol/protocol_engine.hpp"

#include <vector>
#include <complex>
#include <random>
#include <string>
#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <array>
#include <atomic>
#include <condition_variable>

namespace ultra {
namespace gui {

class App {
public:
    // Flags for developer features
    struct Options {
        bool enable_sim = false;      // -sim: Show simulation UI
        bool record_audio = false;    // -rec: Record all audio to file
        bool safe_startup = false;    // Defer heavyweight init (audio/sim) until needed
        bool disable_waterfall = false; // Skip waterfall construction (startup safety)
        std::string record_path = "sim_recording.f32";  // Recording output base path
        std::string ota_host;
        std::string ota_udp_host;
        std::string token;
        std::string station_id;
        std::string session_id = "lobby";
        bool monitor_audio = false;   // -sim: also play RX audio through local speakers
        std::string monitor_device;   // -sim: optional SDL output device name

        // Monitor mode: skip the full PING/CONNECT handshake and force
        // the decoder into a specific waveform/rate. Useful for OTA
        // monitoring (e.g. recording a friend's transmission via
        // WebSDR + replaying through a virtual audio cable into the
        // GUI to see live waterfall + decoded frames).
        // - monitor_mode = "" (default): normal operation
        // - monitor_mode = "ofdm_chirp" / "ofdm_narrow": force connected OFDM
        // - monitor_mode = "mc_dpsk": force MC-DPSK
        std::string monitor_mode;
        std::string monitor_rate = "r1_4";   // Used when monitor_mode is OFDM
        std::string monitor_modulation = "dqpsk";

        // Scenario scripting (additive test/demo automation). Drives the real
        // UI actions (connect/accept/sendFile/sendMessage) from the render loop
        // so the production App path can be exercised headlessly-ish (window
        // still renders) and cross-checked against cli_simulator. NOT a
        // deterministic CI gate — wall-clock paced; cli_simulator keeps that role.
        std::string auto_connect;             // peer callsign to initiate a connection to
        int connect_delay_sec = 0;            // wait N s after startup before auto-connecting
        bool auto_accept = false;             // auto-accept the first incoming call
        std::string auto_send_file;           // file to send once CONNECTED (fires once)
        std::string auto_send_message;        // message to send once CONNECTED (fires once)
        int auto_message_start_delay_sec = 0; // wait N s after CONNECTED before first auto-message
        int auto_message_count = 1;           // how many sequential messages to send (numbered)
        int auto_message_interval_sec = 8;    // gap between sequential auto-messages
        bool auto_message_vary_len = false;   // randomize each auto-message length (mix short+long)
        bool auto_message_after_file = false; // if file+message are set, send message after file clears
        std::string auto_reply_message;       // responder: send this ONCE after expected inbound messages
                                              // (tests bidirectional reply + half-duplex turn reversal)
        int auto_reply_after_messages = 1;    // wait for N inbound messages before the one reply
        int auto_cancel_file_after_sec = 0;   // cancel active file N s after first observing it
        int auto_disconnect_after_sec = 0;    // 0 = never; else grace after payload drain
        // --disconnect-on-file-done: in a one-way caller-as-sender scenario, let the scripted
        // connection initiator end the session as soon as successful sender completion proves
        // the final file ACK was consumed. The responder may also receive this option, but
        // waits for the caller's close so the final ACK cannot trigger a crossed teardown.
        bool disconnect_on_file_done = false;
        int exit_after_sec = 0;               // 0 = never; else push SDL_QUIT after N s
        bool half_duplex_interactive = false; // bidirectional role-swap (both stations send)
    };

    App();  // Default constructor
    explicit App(const Options& opts);
    ~App();

    void render();

private:
    // Widgets
    ConstellationWidget constellation_;
    ControlsWidget controls_;
    StatusWidget status_;
    SettingsWindow settings_window_;
    std::unique_ptr<WaterfallWidget> waterfall_;
    FileBrowser file_browser_;

    // Persistent settings
    AppSettings settings_;

    // Real modem engine
    AudioEngine audio_;
    ModemEngine modem_;

    // BUG-TNC-SESSION-001 fix: track previous modem-connected state so the
    // connection-changed callback can detect transitions to DISCONNECTED and
    // wrap the audio input producer with pause/drain/resume around the
    // decoder reset (see app.cpp connection-changed callback).
    bool was_modem_connected_ = false;

    // Modem state
    ModemConfig config_;
    ultra::ModemStats stats_;  // From types.hpp for status widget
    bool connected_peer_snr_valid_ = false;
    float connected_peer_snr_db_ = 0.0f;

    // S-meter ballistics for the operator SNR meter. The connected meter shows the
    // per-burst in-band SNR (broadband, the documented 3 kHz operator convention), which
    // on a fading channel legitimately swings several dB burst-to-burst; painted raw at
    // frame rate it reads as "jumping / messed up". These hold the EMA-smoothed value and
    // freeze it across between-burst gaps so the meter behaves like a real radio's S-meter.
    // DISPLAY-LAYER ONLY — never feeds rate selection (which uses the raw physical source).
    bool snr_ballistics_valid_ = false;
    float snr_ballistics_db_ = 0.0f;
    SNRSource snr_ballistics_source_ = SNRSource::NONE;
    float snr_ballistics_last_raw_db_ = 0.0f;  // last raw sample folded into the EMA
    bool snr_ballistics_have_raw_ = false;
    int snr_ballistics_frame_ = -1;            // ImGui frame of the last EMA advance

    // Operate mode state
    char tx_text_buffer_[256] = "Hello from ProjectUltra!";
    std::deque<std::string> rx_log_;
    mutable std::mutex rx_log_mutex_;
    // In-memory scrollback for the operator Message Log. The full history is
    // also mirrored to the GUI log file (see appendRxLogLine), so this only
    // bounds what is scrollable/copyable in the UI. 20 was far too small — it
    // dropped almost all history within seconds of a transfer, breaking
    // scroll-back and making Copy capture only the last 20 lines.
    static const size_t MAX_RX_LOG = 5000;
    bool audio_initialized_ = false;
    bool deferred_audio_auto_init_pending_ = false;
    uint32_t deferred_audio_auto_init_deadline_ms_ = 0;
    int deferred_audio_auto_init_attempts_ = 0;
    bool deferred_audio_wait_logged_ = false;
    bool deferred_radio_rx_start_pending_ = false;
    uint32_t deferred_radio_rx_start_deadline_ms_ = 0;
    uint32_t deferred_radio_rx_start_timeout_ms_ = 0;
    int deferred_radio_rx_start_attempts_ = 0;
    uint32_t render_frames_seen_ = 0;
    std::atomic<bool> tx_in_progress_{false};  // Thread-safe TX flag for waterfall control
    std::chrono::steady_clock::time_point tx_end_time_;  // When current TX finishes

    // OFDM carrier-sense TX defer queue (half-duplex collision avoidance). Touched
    // only on the main thread (queueRealTxSamples runs from protocol_.tick(), and
    // flushDeferredTxIfReady() runs from the main render loop after pollRadioRx()),
    // so no locking is needed; only the detector reads are cross-thread (safe).
    enum class DeferredTxKind {
        Samples,
        Frame,
        Burst
    };
    struct DeferredTx {
        DeferredTxKind kind = DeferredTxKind::Samples;
        std::vector<float> samples;
        Bytes frame;
        std::vector<Bytes> frames;
        std::string context;
        bool in_qso_data = false;
        bool expect_full_ofdm_anchor_after_tx = false;
        std::chrono::steady_clock::time_point earliest_flush{};
        uint16_t group_seq = 0;  // §14.27 burst group seq (re-emit stamps descriptor)
        // BUG-MC-RETRY-SPURIOUS fix 4: data_mode_generation_ at defer time. DATA-class
        // audio (in_qso_data) rendered under an OLDER generation is provably
        // undecodable after a mode/rate/CW commit — purged, never flushed (rig E1:
        // a stale R3/4 burst deferred pre-commit was flushed 8 s after the R2/3
        // commit = 9.0 s of noise airtime). Control audio never enters this queue
        // while connected (queueRealTxSamples only defers pre-connection or
        // in_qso_data), so control frames are structurally exempt from the purge.
        uint32_t data_mode_gen = 0;
        // Bound to this exact pre-encode burst.  Never represent this with the
        // encoder's shared one-shot while the request is still CCA-deferred.
        BurstAnchorOptions burst_anchor_options{};
    };
    std::deque<DeferredTx> deferred_tx_;
    std::chrono::steady_clock::time_point deferred_tx_deadline_{};
    // BUG-MC-RETRY-SPURIOUS fix 4: monotonically increasing data-mode generation.
    // Bumped by the DataModeChangedCallback (runs under the ProtocolEngine mutex,
    // possibly off the main thread) whenever the committed (mod, rate, cw) tuple
    // actually CHANGES; read on the main thread at defer time (stamp) and in
    // purgeStaleDeferredDataTx() (drop stale DATA-class entries). Atomic because
    // writer thread != reader thread; the last-seen tuple below is only touched
    // inside the callback (serialized by the engine mutex), so it stays plain.
    std::atomic<uint32_t> data_mode_generation_{0};
    bool data_mode_gen_seen_valid_ = false;
    Modulation data_mode_gen_seen_mod_{};
    CodeRate data_mode_gen_seen_rate_{};
    int data_mode_gen_seen_cw_ = 0;
    static constexpr size_t kMaxDeferredTx = 32;    // bounded memory (drop-oldest)
    static constexpr uint32_t kMaxTxDeferMs = 4000; // ~one max OFDM burst; never deadlock
    static constexpr uint32_t kInQsoDataQuietGuardMs = 2000;
    static constexpr uint32_t kFileCancelAudioDrainMs = 5000;
    std::chrono::steady_clock::time_point file_cancel_audio_drain_until_{};
    bool file_cancel_audio_drain_active_ = false;
    // Cached connection state for the carrier-sense gate. queueRealTxSamples()
    // runs inside protocol_ TX callbacks (during protocol_.tick(), which holds
    // the engine mutex), so it must NOT call protocol_.getState() (re-entrant
    // deadlock). The connection-changed callback updates this atomic instead.
    std::atomic<protocol::ConnectionState> conn_state_cached_{
        protocol::ConnectionState::DISCONNECTED};
    // ConnectionState cannot represent the responder's post-DISCONNECT ACK
    // grace (it deliberately remains CONNECTED). Connection publishes this
    // finer phase so every App TX chokepoint can reject stale DATA/ACK audio
    // while still permitting the DISCONNECT/sentinel-ACK close exchange.
    std::atomic<bool> disconnect_teardown_active_{false};
    // #70 STAGE2: after this station PONGs a PING it is a RESPONDER expecting the
    // initiator's CONNECT (a DATA frame), so the robust bare-chirp PING emit is disarmed
    // (bare_chirp_expected_=false) for the initiator's connect-attempt window — else a
    // badly-faded CONNECT, indistinguishable from a bare PING to the chirp-lock gate, gets
    // re-PONGed instead of decoded (the responder-side analogue of #27). SDL_GetTicks() ms
    // deadline; 0 = not armed. Re-armed by the tick when the window elapses while still
    // DISCONNECTED. Written on the decode thread (PONG) + main thread (tick), so atomic.
    std::atomic<uint32_t> responder_connect_expected_until_ms_{0};

    // Scenario scripting state (see Options auto_* fields). tickScenario() runs
    // each render frame and drives the real UI actions at the right lifecycle
    // points. Wall-clock paced (this is a parity/visual harness, not a
    // deterministic gate).
    bool scenario_active_ = false;          // any auto_* flag set
    bool scenario_started_ = false;         // scenario_start_ armed
    bool scenario_connect_issued_ = false;  // connect() fired once
    bool scenario_payload_sent_ = false;    // full payload sequence dispatched
    bool scenario_message_sent_ = false;    // chat-message phase fired (if any)
    bool scenario_reply_pending_ = false;   // RX observed; retry submission until accepted
    bool scenario_reply_sent_ = false;      // auto-reply submission accepted (if any)
    int scenario_reply_inbound_received_ = 0; // inbound application messages counted toward reply
    int scenario_messages_sent_ = 0;        // count of sequential auto-messages sent
    // Protocol callbacks may run off the GUI thread. Delivery state is published
    // before the bounded operator-event queue, so dropped display events cannot
    // make scenario completion bookkeeping stale.
    std::atomic<int> scenario_messages_delivered_{0}; // scripted local messages ACKed by ARQ
    std::atomic<bool> scenario_reply_delivered_{false}; // scripted auto-reply ACKed by ARQ
    bool scenario_file_started_ = false;    // file phase started (if any)
    bool scenario_file_done_seen_ = false;  // file phase left in-progress (complete/cancel)
    bool scenario_connected_seen_ = false;
    bool scenario_file_cancel_timer_started_ = false;
    bool scenario_file_cancel_issued_ = false;
    bool scenario_disconnect_issued_ = false;
    bool scenario_disconnect_completed_ = false;
    // Connection callbacks can run on the RX/protocol thread.  Publish only a
    // one-bit completion request there; tickScenario owns the scenario state and
    // completion timestamp on the GUI thread.
    std::atomic<bool> scenario_disconnect_complete_pending_{false};
    // A scripted caller that exhausts CONNECT retries has no remaining work.
    // Keep interactive stations open, but let automation flush its terminal
    // log and release the audio device instead of idling to --exit-after.
    bool scenario_terminal_failure_ = false;
    // --disconnect-on-file-done handshake. SET after successful outbound file completion
    // (the callback runs under ProtocolEngineMutex, so it must NOT touch protocol_) and
    // CONSUMED in tickScenario() outside the lock. The consumer enforces caller-only teardown
    // ownership. Atomic because the setter and consumer are different threads.
    std::atomic<bool> file_done_disconnect_pending_{false};
    std::chrono::steady_clock::time_point scenario_start_;
    std::chrono::steady_clock::time_point scenario_connected_first_at_;
    std::chrono::steady_clock::time_point scenario_connected_at_;
    std::chrono::steady_clock::time_point scenario_message_sent_at_;
    std::chrono::steady_clock::time_point scenario_file_done_at_;
    std::chrono::steady_clock::time_point scenario_file_cancel_started_at_;
    std::chrono::steady_clock::time_point scenario_disconnect_at_;  // when scripted disconnect issued
    std::chrono::steady_clock::time_point scenario_disconnect_completed_at_;
    std::chrono::steady_clock::time_point scenario_terminal_failure_at_;
    void tickScenario();

    // Modem/protocol callbacks can run on the RX decode / ARQ path. They must
    // never block behind GUI rendering or log-file I/O, so they only enqueue
    // bounded best-effort operator events. The GUI thread drains and formats.
    enum class OperatorEventKind {
        LogLine,
        MessageReceived,
        MessageTxStatus
    };
    struct OperatorEvent {
        OperatorEventKind kind = OperatorEventKind::LogLine;
        std::string line;
        std::string from;
        std::string text;
        protocol::ProtocolEngine::MessageTxStatusEvent tx_status;
    };
    std::deque<OperatorEvent> operator_event_queue_;
    std::mutex operator_event_mutex_;
    std::atomic<uint64_t> operator_events_dropped_{0};
    std::atomic<uint64_t> operator_events_log_lines_{0};
    std::atomic<uint64_t> operator_events_rx_messages_{0};
    std::atomic<uint64_t> operator_events_tx_submitted_{0};
    std::atomic<uint64_t> operator_events_tx_delivered_{0};
    std::atomic<uint64_t> operator_events_tx_failed_{0};

    // Latest in-band decode SNR, cached lock-free off the modem RX path
    // (after_rx_data hook) so the tone-burst ACK callback can pick its §15.5
    // staircase symbol duration WITHOUT calling protocol_ — the ACK callback is
    // invoked while protocol_ holds ProtocolEngineMutex, and getMeasuredSNR()
    // re-locks it, so a direct call self-deadlocks. Atomics → no lock either side.
    std::atomic<float> cached_inband_snr_db_{12.0f};
    std::atomic<SNRSource> cached_inband_snr_source_{SNRSource::NONE};
    // Fading index cached alongside the SNR (same lock-free discipline): the
    // §15.5 staircase fast edge is basis-dependent — fade-effective SNR uses the
    // 16 dB edge, AWGN the 18 dB one (BUG-ACK-STAIRCASE-FADE-BIN, 2026-07-01).
    std::atomic<float> cached_fading_index_{0.0f};
    // BUG-STAIRCASE-SNAPSHOT-INPUT fix (ULTRA_ACK_SNR_MEDIAN, 2026-07-05): ring of
    // the last 5 cached readings; the staircase reads the MEDIAN. A single
    // erasure-slot/trough snapshot (F98: one 4/5 partial wrote 9.4 over a 22 dB
    // channel → 50 ms ACK bin → phantom demote → 54 s epoch saga) cannot move a
    // median-of-5, while a genuine SNR decline still tracks within ~3 groups.
    // Lock-free per the same discipline (independent atomics; ordering slack
    // between slots is harmless for a median).
    std::array<std::atomic<float>, 5> cached_snr_ring_{
        {12.0f, 12.0f, 12.0f, 12.0f, 12.0f}};
    std::atomic<uint32_t> cached_snr_ring_idx_{0};
    // ACK repeat-if-silent (ULTRA_ACK_REPEAT_SILENT_MS, 2026-07-05; handoff §7.6):
    // the stashed decorrelated tone-ACK copy + its fire time. Written on the
    // protocol callback (stash), consumed on the GUI tick (CCA-gated fire) —
    // guarded by its own mutex, never held across protocol_ calls.
    std::mutex ack_repeat_mutex_;
    std::vector<float> ack_repeat_samples_;
    std::chrono::steady_clock::time_point ack_repeat_fire_time_{};
    bool ack_repeat_pending_ = false;
    int64_t ack_repeat_armed_rx_ms_ = 0;  // F143: decoder stamp AT arm; cancel only on NEWER evidence
    int64_t ack_repeat_armed_signal_ms_ = 0;  // broad-stamp baseline; same group must not self-hold
    void maybeFireAckRepeatIfSilent();
    // LISTEN-BEFORE-ACK (2026-07-05, F124 finding: 4 tone-ACKs keyed while the
    // sender's audio was actively arriving — a half-duplex violation that blanks
    // our own RX of the incoming group head and manufactures craters; the primary
    // ACK path had no channel sense at all, only the repeat did). If CCA reads
    // busy at ACK time, defer submission to the GUI tick. Decoder/geometry evidence
    // makes an asynchronous ACK drop at its deadline; energy-only busy may send at
    // the deadline because the adaptive floor can falsely classify ambient noise.
    // Same mutex discipline as the repeat stash.
    std::vector<float> ack_defer_samples_;
    std::chrono::steady_clock::time_point ack_defer_deadline_{};
    bool ack_defer_pending_ = false;
    int ack_defer_quiet_ticks_ = 0;
    bool ack_defer_inbound_group_complete_ = false;
    void maybeFireDeferredAck();
    void submitToneAckSamples(const std::vector<float>& samples);
    void setDisconnectTeardownActive(bool active);
    void clearPendingAckAudioForTeardown();
    std::vector<float> ack_repeat_last_armed_samples_;  // F222: one repeat per distinct ack

    // ── Software-ALC sender state (BUG-QAM16-RIG-LEVEL-BUDGET, 2026-07-02) ──
    // Closed-loop TX-drive: the peer's per-burst level verdict rides back on the
    // tone-burst ACK (drive_advisory bits [30..31]); handleDriveAdvisory() walks
    // this per-connection peak target within [settings_.tx_drive baseline,
    // kSoftwareAlcMaxPeakTarget]. Applied ONLY to connected OFDM data bursts
    // (context "TX burst audio") — handshake/control/MC-DPSK keep the configured
    // baseline. Written on the protocol-callback thread, read on the main TX
    // thread → atomic. Reset to baseline on CONNECTED and DISCONNECTED
    // transitions (per-connection scope).
    std::atomic<float> alc_tx_drive_{ultra::sim::kHardwareTxDefaultPeakTarget};
    // Repeat-ACK dedup (one adjustment per ACKed group): last group_seq/advisory
    // acted on. Written by handleDriveAdvisory (tone-burst detection path) and
    // resetSoftwareAlc (connection-state callback); the two never overlap in
    // practice (no in-flight data ACKs at a CONNECT/DISCONNECT edge) and the worst
    // possible race outcome is one duplicate/missed ±step, bounded by the clamps —
    // plain fields are fine.
    int alc_last_group_seq_ = -1;
    uint8_t alc_last_advisory_ = 0;
    void handleDriveAdvisory(uint8_t advisory, uint8_t group_seq);
    void resetSoftwareAlc(const char* reason);
    // Effective per-burst peak target for this TX context: the ALC-walked drive
    // for connected OFDM data bursts, the configured baseline for everything else.
    float effectiveTxDriveForContext(const char* context) const;

    bool operator_log_file_suppressed_ = false;
    uint32_t operator_log_slow_ms_ = 0;
    size_t operator_event_drain_limit_ = 128;
    static constexpr size_t MAX_OPERATOR_EVENTS = 1024;
    size_t operator_event_queue_limit_ = MAX_OPERATOR_EVENTS;
    void enqueueOperatorEvent(OperatorEvent event);
    void enqueueOperatorLogLine(std::string line);
    void enqueueMessageReceived(std::string from, std::string text);
    void enqueueMessageTxStatus(protocol::ProtocolEngine::MessageTxStatusEvent event);
    void drainOperatorEvents(bool flush_all = false);
    void appendRxLogLineNow(const std::string& msg);
    void logOperatorEventStats(const char* reason);

    // Radio mode state
    std::vector<std::string> input_devices_;
    std::vector<std::string> output_devices_;
    bool ptt_active_ = false;      // Push-to-talk state
    bool ptt_release_pending_ = false;
    uint32_t ptt_release_deadline_ms_ = 0;
    bool radio_rx_enabled_ = false; // RX capture running
    uint32_t radio_rx_started_ms_ = 0;
    bool radio_rx_warmup_logged_ = false;
    bool radio_rx_first_chunk_logged_ = false;
    uint32_t radio_rx_no_data_deadline_ms_ = 0;
    int radio_rx_rearm_attempts_ = 0;
    bool radio_rx_rearm_exhausted_logged_ = false;
    std::string radio_rx_active_device_;
    bool radio_rx_force_queue_mode_ = false;
    bool radio_rx_output_prime_attempted_ = false;

    // ARQ Protocol state
    protocol::ProtocolEngine protocol_;
    std::unique_ptr<ptt::IPttDriver> ptt_driver_;
    ptt::PttConfig ptt_config_;
    std::mutex ptt_driver_mutex_;
    uint32_t cat_frequency_next_open_attempt_ms_ = 0;
    char remote_callsign_[16] = "";
    std::string pending_incoming_call_;  // Callsign of incoming caller
    uint32_t last_tick_time_ = 0;

    // File transfer state
    char file_path_buffer_[512] = "";
    std::string last_received_file_;  // Path of last received file
    bool file_browser_open_ = false;
    int last_progress_milestone_ = 0;  // Last logged milestone (0, 25, 50, 75)
    std::chrono::steady_clock::time_point file_transfer_start_time_;  // For duration display
    bool rx_transfer_clock_armed_ = false;  // RX throughput clock started (at first burst, not FILE_START)
    uint32_t pending_file_tx_payload_bytes_ = 0;  // Original file size for TX goodput
    std::string pending_file_tx_path_;
    float last_effective_goodput_bps_ = 0.0f;  // Last completed file transfer goodput
    std::string last_goodput_label_ = "n/a";  // Context for last goodput sample
    int diagnostics_last_arq_failed_ = 0;
    std::string image_send_path_;
    ImageInfo image_send_info_;
    int image_send_preset_ = 0;  // 0 thumbnail, 1 preview, 2 full size
    std::string image_send_error_;
    bool send_btn_log_valid_ = false;
    bool send_btn_log_enabled_ = false;
    bool send_btn_log_scenario_ = false;
    bool send_btn_log_file_busy_ = false;
    bool send_btn_log_tx_inprog_ = false;
    size_t send_btn_log_textlen_ = 0;
    size_t send_btn_log_backlog_bytes_ = 0;
    bool send_btn_log_connected_ = false;
    bool send_btn_log_would_enable_ = false;

    bool startFileSend(const std::string& file_path, const std::string& success_log);
    void startFileSendOrImageDialog(const std::string& file_path);
    void renderImageSendModal();

    // Diagnostics report UI
    bool diagnostics_consent_popup_opened_ = false;
    bool diagnostics_report_popup_open_ = false;
    bool diagnostics_debrief_popup_open_ = false;
    char diagnostics_note_buffer_[512] = "";
    char diagnostics_debrief_save_path_[512] = "";
    std::string diagnostics_last_report_;
    std::string diagnostics_last_summary_;
    std::string diagnostics_last_summary_path_;
    std::string diagnostics_debrief_status_;
    void renderDiagnosticsDialogs();

    // ========================================
    // Developer Options
    // ========================================
    Options options_;                           // Command-line options

    bool simulation_enabled_ = false;           // -sim: OTASim server client mode

    // Audio recording (requires -rec flag)
    bool recording_enabled_ = false;            // Currently recording
    std::vector<float> recorded_rx_samples_;    // Real RX audio fed to modem
    std::vector<float> recorded_tx_samples_;    // Real TX audio queued to output
    void writeRecordingToFile();                // Save recording buffers to disk
    std::unique_ptr<ultra::otasim_client::OtaAudioBackend> ota_audio_;
    uint32_t ota_monitor_device_id_ = 0;  // SDL_AudioDeviceID; 0 = closed

    // ========================================
    // UI Rendering
    // ========================================
    void renderOperateTab();
    void renderCompactChannelStatus(const LoopbackStats& stats, Modulation data_mod, CodeRate data_rate,
                                    const protocol::ConnectionStats& conn_stats);
    // Apply S-meter ballistics (EMA smooth + hold-across-gaps) to the operator SNR meter
    // while connected; pass the raw reading through unchanged when idle. Advances the EMA
    // at most once per ImGui frame and only on a NEW measurement, so the per-burst sample
    // (held constant between bursts) is smoothed instead of snapping. Display-layer only.
    OperatorSNRDisplay updateSnrBallistics(const OperatorSNRDisplay& raw, bool connected);
    void initAudio();
    void initOtaAudio();
    void stopOtaAudio();
    void pollOtaRx();
    void appendRxLogLine(const std::string& msg);
    std::deque<std::string> snapshotRxLog() const;
    void clearRxLog();
    void stopTxNow(const char* reason);
    void cancelActiveFileTransfer();
    void beginFileCancelAudioDrain(const char* reason);
    bool isFileCancelAudioDrainActive();
    // OFDM carrier-sense TX gate (half-duplex collision avoidance). queueRealTxSamples
    // is the single chokepoint all TX paths funnel through; in OFDM mode it defers the
    // burst when the peer is still on-channel instead of keying up over it.
    // flushDeferredTxIfReady() drains the deferred queue on the main thread once the
    // channel goes idle, bounded by kMaxTxDeferMs so a stuck-busy reading never
    // deadlocks TX. doQueueRealTxSamples() is the real key-up + send (bypasses the gate).
    bool queueRealTxSamples(const std::vector<float>& samples, const char* context,
                            bool in_qso_data = false,
                            bool disconnect_control = false);
    bool doQueueRealTxSamples(const std::vector<float>& samples, const char* context,
                              bool disconnect_control = false);
    bool shouldDeferInQsoDataForTx() const;
    void deferTxSamples(const std::vector<float>& samples, const char* context,
                        bool in_qso_data,
                        std::chrono::steady_clock::time_point earliest_flush = {});
    void deferTxFrame(const Bytes& frame, const char* context,
                      bool expect_full_ofdm_anchor_after_tx);
    void deferTxBurst(const std::vector<Bytes>& frames, const char* context,
                      uint16_t group_seq = 0,
                      BurstAnchorOptions anchor_options = {});
    uint32_t nextInQsoDataBackoffMs();
    void flushDeferredTxIfReady();
    // BUG-MC-RETRY-SPURIOUS fix 4: drop deferred DATA-class TX rendered under a
    // superseded data-mode generation (undecodable by the peer after the commit).
    // Runs at the top of flushDeferredTxIfReady() on the main thread.
    void purgeStaleDeferredDataTx();
    ptt::PttConfig pttConfigFromSettings(const AppSettings& settings) const;
    bool ensurePttReadyLocked(const AppSettings& settings);
    bool ensurePttReady();
    bool setPtt(bool asserted, const char* reason);
    void releasePtt(const char* reason);
    void closePtt();
    std::string testPtt(AppSettings settings);
    std::string testCat(AppSettings settings);
    void updateWaterfallFrequencyDisplay();
    void sendMessage();
    void onDataReceived(const std::string& text);
    bool startRadioRx();
    void stopRadioRx();
    void pollRadioRx();

    // Helper to get device name from settings (returns empty string for "Default")
    std::string getInputDeviceName() const;
    std::string getOutputDeviceName() const;
};

} // namespace gui
} // namespace ultra
