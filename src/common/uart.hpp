// Ported from uliano/AVR-Multislope (lib/core/src/uart.hpp).
/*
 * uart.hpp
 *
 * Interrupt-driven UART byte transport.
 *
 * Created: 11/3/2022 4:58:21 PM
 *  Author: uliano
 * Revised: 01/9/2026 - Transport/parser decoupling and cleanup
 */


#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>
#include "bytestream.hpp"
#include "ring.hpp"
#include "utils.hpp"
#include "ticker.hpp"

const bool UART_ALTERNATE = true;
const bool UART_STANDARD = false;

/**
 * @brief Interrupt-driven UART transport implementing ByteStream
 *
 * @tparam uart_num UART peripheral number (0-5)
 * @tparam alternate_pins Use alternate pin mapping (see datasheet)
 * @tparam rsize RX buffer size (power of 2)
 * @tparam tsize TX buffer size (power of 2)
 * @tparam RSizeT RX index type (uint8_t/uint16_t)
 * @tparam TSizeT TX index type (uint8_t/uint16_t)
 *
 * Key features:
 * - Byte-oriented RX/TX buffers (power-of-2 ring buffers)
 * - Interrupt-driven TX: Non-blocking transmission with DRE interrupt
 * - Error counters: TX/RX overflow (wrap at 255)
 *
 * Usage:
 *   Uart<2, UART_ALTERNATE> usb(115200);
 *
 *   // In ISR (interrupts.cpp):
 *   ISR(USART2_RXC_vect) { usb.rxc(); }
 *   ISR(USART2_DRE_vect) { usb.dre(); }
 *
 *   // In main loop:
 *   uint8_t b;
 *   if (usb.read_byte(b)) {
 *       // Consume received byte
 *   }
 */
template <const int uart_num,  bool alternate_pins=false, const int rsize=256, const int tsize=512, typename RSizeT=uint8_t, typename TSizeT=uint16_t>
class Uart : public ByteStream {
	static_assert((uart_num >= 0) && (uart_num <= 5), "uart can be only 0, 1, 2, 3, 4, or 5");
	static_assert(is_powerof2(rsize), "uart read buffer should be a power of 2");
	static_assert(is_powerof2(tsize), "uart write buffer should be a power of 2");
	private:
	volatile USART_t *regs;
	Ring<uint8_t, RSizeT, rsize> m_input_buffer;
	Ring<uint8_t, TSizeT, tsize> m_output_buffer;

	// Error counters (volatile uint8_t for atomic ISR access, wrap at 255).
	volatile uint8_t m_tx_errors;
	volatile uint8_t m_rx_errors;

	public:
    using ByteStream::write;

	Uart(uint32_t baud=115200)
	{
		m_tx_errors = 0;
		m_rx_errors = 0;
        if constexpr (uart_num == 0) {
            regs = &USART0;
            if constexpr (alternate_pins) {
                PORTMUX.USARTROUTEA |= PORTMUX_USART0_0_bm;
                PORTA.DIRSET = PIN4_bm; // Tx
                PORTA.DIRCLR = PIN5_bm; // Rx
                PORTA.OUTSET = PIN4_bm | PIN5_bm; // initial state HI
            } else {
                PORTA.DIRSET = PIN0_bm; // Tx
                PORTA.DIRCLR = PIN1_bm; // Rx
                PORTA.OUTSET = PIN0_bm | PIN1_bm; // initial state HI
            }
        } else if constexpr (uart_num == 1) {
            regs = &USART1;
            if constexpr (alternate_pins) {
                PORTMUX.USARTROUTEA |= PORTMUX_USART1_0_bm;
                PORTC.DIRSET = PIN4_bm; // Tx
                PORTC.DIRCLR = PIN5_bm; // Rx
                PORTC.OUTSET = PIN4_bm | PIN5_bm; // initial state HI
            } else {
                PORTC.DIRSET = PIN0_bm; // Tx
                PORTC.DIRCLR = PIN1_bm; // Rx
                PORTC.OUTSET = PIN0_bm | PIN1_bm; // initial state HI
            }
        }
        #ifdef USART2
        else if constexpr (uart_num == 2) {
            regs = &USART2;
            if constexpr (alternate_pins) {
                PORTMUX.USARTROUTEA |= PORTMUX_USART2_0_bm;
                PORTF.DIRSET = PIN4_bm; // Tx
                PORTF.DIRCLR = PIN5_bm; // Rx
                PORTF.OUTSET = PIN4_bm | PIN5_bm; // initial state HI
            } else {
                PORTF.DIRSET = PIN0_bm; // Tx
                PORTF.DIRCLR = PIN1_bm; // Rx
                PORTF.OUTSET = PIN0_bm | PIN1_bm; // initial state HI
            }
        }
        #endif
        #ifdef USART3
        else if constexpr (uart_num == 3) {
            regs = &USART3;
            if constexpr (alternate_pins) {
                PORTMUX.USARTROUTEA |= PORTMUX_USART3_0_bm;
                PORTB.DIRSET = PIN4_bm; // Tx
                PORTB.DIRCLR = PIN5_bm; // Rx
                PORTB.OUTSET = PIN4_bm | PIN5_bm; // initial state HI
            } else {
                PORTB.DIRSET = PIN0_bm; // Tx
                PORTB.DIRCLR = PIN1_bm; // Rx
                PORTB.OUTSET = PIN0_bm | PIN1_bm; // initial state HI
            }
        }
        #endif
        #ifdef USART4
        else if constexpr (uart_num == 4) {
            regs = &USART4;
            if constexpr (alternate_pins) {
                PORTMUX.USARTROUTEB |= PORTMUX_USART4_0_bm;
                PORTE.DIRSET = PIN4_bm; // Tx
                PORTE.DIRCLR = PIN5_bm; // Rx
                PORTE.OUTSET = PIN4_bm | PIN5_bm; // initial state HI
            } else {
                PORTE.DIRSET = PIN0_bm; // Tx
                PORTE.DIRCLR = PIN1_bm; // Rx
                PORTE.OUTSET = PIN0_bm | PIN1_bm; // initial state HI
            }
        }
        #endif
        #ifdef USART5
        else if constexpr (uart_num == 5) {
            regs = &USART5;
            if constexpr (alternate_pins) {
                PORTMUX.USARTROUTEB |= PORTMUX_USART5_0_bm;
                PORTG.DIRSET = PIN4_bm; // Tx
                PORTG.DIRCLR = PIN5_bm; // Rx
                PORTG.OUTSET = PIN4_bm | PIN5_bm; // initial state HI
            } else {
                PORTG.DIRSET = PIN0_bm; // Tx
                PORTG.DIRCLR = PIN1_bm; // Rx
                PORTG.OUTSET = PIN0_bm | PIN1_bm; // initial state HI
            }
        }
        #endif

		regs->CTRLA |= USART_RXCIE_bm;
		// regs->BAUD = uint16_t(float(F_CPU * 64 / (16 * float(baud)) + 0.5));
		regs->BAUD = uint16_t((F_CPU * 4UL + baud / 2UL) / baud);
		regs->CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
		_delay_ms(10); // give a little time with TX high to have a proper start bit
	}

    /**
     * @brief TX Data Register Empty interrupt handler
     *
     * Call this from ISR(USARTx_DRE_vect).
     * Sends next byte from TX buffer, disables interrupt when empty.
     */
    inline void dre(void) {
        uint8_t c;
        if (m_output_buffer.get_from_isr(c)) {  // Attempt to get a byte
            regs->TXDATAL = c;
            if (!m_output_buffer.size_from_isr()) {  // Disable DRE interrupt if empty
                regs->CTRLA &= ~USART_DREIE_bm;
            }
        } else {
            // Buffer was empty, so disable the DRE interrupt
            regs->CTRLA &= ~USART_DREIE_bm;
        }
    }

	/**
	 * @brief RX Complete interrupt handler
	 *
	 * Call this from ISR(USARTx_RXC_vect).
	 * Receives one byte and pushes it into the RX ring buffer.
	 */
	inline void rxc(void) {
		uint8_t value =regs->RXDATAL;
		if (!m_input_buffer.try_put_from_isr(value)) {
			++m_rx_errors;  // Atomic - uint8_t on 8-bit AVR
		}
	}

    bool read_byte(uint8_t &b) override {
        return m_input_buffer.get(b);
    }

    inline bool receive_byte(uint8_t *b) {
        if (!b) return false;
        return read_byte(*b);
    }

	inline uint8_t tx_errors(void) const {
		return m_tx_errors;
	}

	inline uint8_t rx_errors(void) const {
		return m_rx_errors;
	}

	inline RSizeT rx_size(void) const {
		return m_input_buffer.size();
	}

	inline void clear_errors(void) {
		m_tx_errors = 0;
		m_rx_errors = 0;
	}

    bool write_byte(uint8_t b) override {
		if (!m_output_buffer.try_put(b)) {
			++m_tx_errors;  // Atomic - uint8_t on 8-bit AVR
			return false;
		}
		regs->CTRLA |= USART_DREIE_bm;
		return true;
    }

	inline uint8_t send_byte(const uint8_t b){
		return write_byte(b) ? 1 : 0;
	}

    uint8_t write(const uint8_t *buffer, uint8_t len) override {
		uint8_t remaining = len;
		while(remaining)
		{
			const bool result = write_byte(*buffer);
			if (result) {
				++buffer;
				--remaining;
			}
			else {
				break;
			}
		}
		return len - remaining;
	}

	uint8_t send_buffer(const uint8_t *buffer, const uint8_t len){
        return write(buffer, len);
	}

	inline void newline(const bool cr=false) {
		if (cr) print("\r\n");
		else print("\n");
	}

	void print(const char *string) {
		while(*string) {
			send_byte(*string);
			++string;
		}
	}

	void print(const double val, int8_t width, uint8_t precision) {
		uint8_t buffer[16];
		dtostrf(val, width, precision, buffer);
		print(buffer);
	}

	void print(const double val, uint8_t precision=3) {
		uint8_t buffer[16];
		dtostre(val, buffer, precision, DTOSTR_ALWAYS_SIGN);
		print(buffer);
	}

	void print(const uint32_t val, int8_t radix){
		char buffer[12];
		if (radix==16){
			buffer[0] = '0';
			buffer[1] = 'x';
			ultoa(val, &buffer[2], radix);
		} else {
			ultoa(val, buffer, radix);
		}
		print(buffer);
	}

	void print(const int32_t val){
		char buffer[12];
		ltoa(val, buffer, 0x0a);
		print(buffer);
	}

	void print(const uint16_t val, int8_t radix){
		char buffer[8];
		if (radix==16){
			buffer[0] = '0';
			buffer[1] = 'x';
			ultoa(val, &buffer[2], radix);
		} else {
			ultoa(val, buffer, radix);
		}
		print(buffer);
	}

	void print(const int16_t val){
		char buffer[8];
		itoa(val, buffer, 0x0a);
		print(buffer);
	}

	inline void print(const uint8_t u, int8_t radix) {
		print(static_cast<uint16_t>(u), radix);
	}

	inline void print(const int8_t u) {
		print(static_cast<int16_t>(u));
	}

	void print(TimeStamp & t){
		print(t.seconds, 10);
		print("s.");
		print(t.ticks, 10);
		print("t");
	}

}; //Uart
