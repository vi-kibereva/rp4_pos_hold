#include <cstdint>
#include <cstring>
#include <iostream>
#include <termios.h>

#include "msp/bitaflught_msp.hpp"
#include "msp/serial_stream.hpp"

namespace msp {

BitaflughtMsp::BitaflughtMsp(const char *dev, speed_t baud_rate, cc_t timeout)
		: stream_(dev, baud_rate, timeout) {}

BitaflughtMsp::~BitaflughtMsp() = default;

void BitaflughtMsp::send(CommandType command_type, std::uint8_t command_id,
						 const void *payload, std::uint8_t size) {
	uint8_t frame[5 + 255 + 1];
	frame[0] = '$';
	frame[1] = 'M';
	frame[2] = msp::to_underlying(command_type);
	frame[3] = size;
	frame[4] = command_id;

	const uint8_t *p = static_cast<const uint8_t *>(payload);
	uint8_t chk = static_cast<uint8_t>(size ^ command_id);

	for (uint8_t i = 0; i < size; ++i) {
		uint8_t b = p[i];
		frame[5 + i] = b;
		chk ^= b;
	}

	frame[5 + size] = chk;

	const size_t total = 5 + size + 1;
	stream_.write(frame, total);
}

bool BitaflughtMsp::recv(std::uint8_t *command_id, void *payload,
						 std::uint8_t max_size, std::uint8_t *recv_size) {
	uint8_t buffer[5 + 255 + 1];
	uint8_t size_b = 0;
	while (true) {
		size_t a;
		while (stream_.available() < 5) {
		}
		size_b += stream_.read(buffer + size_b, 5);
		if (buffer[0] == '$' && buffer[1] == 'M' && buffer[2] == '>') {
			*recv_size = buffer[3];
			*command_id = buffer[4];
			std::uint8_t checksumCalc = *recv_size ^ *command_id;
			auto *payload_ptr = static_cast<uint8_t *>(payload);
			while (stream_.available() < *recv_size + 1) {
			}
			size_b += stream_.read(buffer + size_b, *recv_size + 1);
			for (int i = 0; i < *recv_size; i++) {
				uint8_t b = buffer[i + 5];
				checksumCalc ^= b;
				*(payload_ptr++) = b;
			}
			uint8_t checksum = buffer[5 + *recv_size];
			for (std::uint8_t j = *recv_size + 6; j < max_size; ++j) {
				*(payload_ptr++) = 0;
			}
			if (checksum == checksumCalc) {
				return true;
			}
			std::cout << "Invalid checksum (calc): " << static_cast<int>(checksumCalc)
								<< "; (received): " << static_cast<int>(checksum) << std::endl;
		}
		std::cout << "Wrong header" << buffer[0] << buffer[1] << buffer[2]
							<< std::endl;
		return false;
	}
}

bool BitaflughtMsp::request(std::uint8_t command_id, void *payload,
							std::uint8_t max_size, std::uint8_t *recv_size) {
	send(CommandType::Request, command_id, NULL, 0);
	return waitFor(command_id, payload, max_size, recv_size);
}

void BitaflughtMsp::reset() { stream_.flush(); }

bool BitaflughtMsp::waitFor(std::uint8_t command_id, void *payload,
							std::uint8_t max_size, std::uint8_t *recv_size) {
	std::uint8_t rx_id = 0;
	std::uint8_t out_len;

	while (true) {
		if (!recv(&rx_id, payload, max_size, (recv_size ? recv_size : &out_len))) {
			if (recv_size)
				*recv_size = 0;
			return false;
		}

		if (rx_id == command_id)
			return true;

		std::cout << "wrong command_id: " << static_cast<int>(rx_id) << std::endl;
	}
}

bool BitaflughtMsp::command(std::uint8_t command_id, void *payload,
							std::uint8_t size, bool wait_ACK) {
	send(CommandType::Request, command_id, payload, size);

	if (wait_ACK)
		return waitFor(command_id, nullptr, 0);
	return true;
}
bool BitaflughtMsp::getActiveModes(std::uint32_t *active_modes) {
	(void)active_modes;

	return true;
}

} // namespace msp
