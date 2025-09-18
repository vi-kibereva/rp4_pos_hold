#ifndef RPI_MSP_HPP
#define RPI_MSP_HPP

#include <cstdint>

#include "serial_iostream.hpp"

namespace msp {

enum class MspVersion : std::uint8_t { MSP_V1, MSP_V2 };

enum class MspCommands : std::uint8_t {
  MSP_VERSION_MAJOR = 1,
};

void begin();

/**
 * @brief High-level helper for exchanging MSP (MultiWii Serial Protocol) frames
 *        over an iostream-like byte stream.
 *
 * Responsibilities:
 *  - Encode and transmit MSP frames (header, size, id, payload, checksum).
 *  - Receive, validate (checksum), and parse MSP frames.
 *  - Provide convenience round-trips (request/response) and simple queries.
 *
 * @par Ownership & lifetime
 * The manager holds a **reference** to an external @ref serial_iostream.
 * The caller must ensure the referenced stream outlives this manager.
 *
 * @par Thread safety
 * Not thread-safe. Synchronize externally if used from multiple threads.
 *
 * @par Errors & timeouts
 * Methods that return `bool` report success (`true`) or failure (`false`)
 * due to I/O errors, protocol violations, or timeouts. No exceptions are
 * thrown.
 */
class msp_manager {
public:
  /// Deleted: a stream is required to operate.
  msp_manager() = delete;

  /**
   * @brief Bind to an already-opened serial stream.
   *
   * @param serial_stream Stream configured for binary I/O to the MSP peer.
   *
   * The manager stores a reference to @p serial_stream and does not take
   * ownership. Use @ref begin() to perform initial protocol setup (e.g., clear
   * residual bytes).
   *
   * @warning Because @ref stream_ is a reference member, passing a temporary
   *          or a soon-to-be-destroyed object leads to dangling references.
   */
  explicit msp_manager(msp::serial_iostream &serial_stream);

  /// Virtual-free: does not close the underlying stream.
  ~msp_manager();

  // ------------------------- Special members -------------------------

  /// Non-copyable (reference member).
  msp_manager(const msp_manager &) = delete;
  /// Non-assignable (reference member).
  msp_manager &operator=(const msp_manager &) = delete;
  /// Non-movable (reference member).
  msp_manager(msp_manager &&) = delete;
  /// Non-movable (reference member).
  msp_manager &operator=(msp_manager &&) = delete;

  // --------------------------- Operations ----------------------------

  /**
   * @brief Send an MSP frame (request/command).
   *
   * @param message_id MSP message code.
   * @param payload    Pointer to payload bytes (may be nullptr if @p size ==
   * 0).
   * @param size       Payload size in bytes.
   *
   * @post A complete MSP frame is written to the stream (no implicit response
   * wait).
   */
  void send(uint8_t message_id, void *payload, uint8_t size);

  /**
   * @brief Send an MSP error frame for a given @p message_id.
   *
   * @param message_id Message code that failed.
   * @param payload    Optional error payload (dialect-specific).
   * @param size       Payload size.
   *
   * @details Use only if your MSP dialect defines error replies.
   */
  void error(uint8_t message_id, void *payload, uint8_t size);

  /**
   * @brief Send an MSP response frame for a given @p message_id.
   *
   * @param message_id Request code being answered.
   * @param payload    Response payload bytes.
   * @param size       Payload size.
   *
   * @details Typical when acting as an MSP target/server.
   */
  void response(uint8_t message_id, void *payload, uint8_t size);

  /**
   * @brief Receive one MSP frame matching @p message_id.
   *
   * @param message_id Expected MSP code.
   * @param payload    Output buffer to receive bytes.
   * @param max_size   Capacity of @p payload.
   * @param recv_size  [out] Actual bytes written (optional).
   * @return true on success; false on timeout, checksum error, or I/O failure.
   *
   * @note Frames with other IDs may be consumed internally while searching.
   */
  bool recv(uint8_t message_id, void *payload, uint8_t max_size,
            uint8_t *recv_size);

  /**
   * @brief Wait for an incoming frame with @p message_id (no transmit).
   *
   * @param message_id Expected MSP code.
   * @param payload    Output buffer.
   * @param max_size   Capacity of @p payload.
   * @param recv_size  [out] Actual bytes received (optional).
   * @return true on success; false otherwise.
   *
   * @note The name appears misspelled as `wairFor`; consider renaming to
   * `waitFor`.
   */
  bool wairFor(uint8_t message_id, void *payload, uint8_t max_size,
               uint8_t *recv_size = nullptr);

  /**
   * @brief Send a request and wait for its response.
   *
   * @param message_id MSP code to request.
   * @param payload    Output buffer for the response payload.
   * @param max_size   Capacity of @p payload.
   * @param recv_size  [out] Actual response size (optional).
   * @return true on success; false on timeout/protocol/transport error.
   *
   * @post Equivalent to @ref send(...) followed by @ref recv(...).
   */
  bool request(uint8_t message_id, void *payload, uint8_t max_size,
               uint8_t *recv_size = nullptr);

  /**
   * @brief Send a command with payload; optionally wait for an acknowledgement.
   *
   * @param message_id MSP code.
   * @param payload    Command payload.
   * @param size       Payload size.
   * @param wait_ack   If true, block until an ACK/expected reply is observed.
   * @return true on success (and ACK if requested); false otherwise.
   *
   * @details The “ACK” semantics depend on the MSP dialect (could be same
   *          message_id echoed or a dedicated status code).
   */
  bool command(uint8_t message_id, void *payload, uint8_t size,
               bool wait_ack = true);

  /**
   * @brief Reset internal parser/driver state to defaults.
   *
   * @post Discards partial frames and restores default timeout.
   */
  void reset();

  /**
   * @brief Convenience query: get active modes bitmask from the peer.
   *
   * @param active_modes [out] Bitmask of active modes (device-specific).
   * @return true on success; false otherwise.
   */
  bool get_active_modes(uint32_t *active_modes);

  /**
   * @brief Initialize the manager/transport before first use.
   *
   * Typical tasks:
   *  - verify stream readiness,
   *  - set default timeout,
   *  - clear residual bytes,
   *  - (optionally) query MSP API/version/capabilities.
   */
  void begin();

private:
  /// Underlying byte stream (not owned). Must outlive this object.
  msp::serial_iostream &stream_;

  /// Operation timeout in milliseconds for blocking calls.
  uint32_t timeout_{1000};
};

} // namespace msp

#endif // RPI_MSP_HPP
