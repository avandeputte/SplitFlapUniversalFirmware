; =============================================================================
; Split-Flap module bootloader: the bus at 9600 baud, 8N1, bit-banged
; =============================================================================
; In assembly so that every bit takes a known number of cycles at 10 MHz:
; 1041.7 would be exact; these take 1043 to receive and 1046-1047 to send,
; well inside what a UART tolerates.  Called from bootloader.c (avr-gcc ABI:
; argument and result in r24; r23-r25 are free to use; r1 stays 0).
; =============================================================================
#include <avr/io.h>

#define RX_BIT          7               /* PA7, from the transceiver's RO */
#define TX_BIT          5               /* PA5, to its DI */
#define HALF_BIT_LOOPS  171             /* half_bit: 3n+6 = 519 cycles with its rcall */

        .section .text

; Half a bit.  Uses r25.
        .global half_bit
half_bit:
        ldi     r25, HALF_BIT_LOOPS
1:      dec     r25
        brne    1b
        ret

; Wait for a byte and return it in r24.  A start bit must still be low half a
; bit after its edge, so a glitch on an idle line is not taken for a byte.
; Returns once the stop bit has begun, so the caller has a bit's time before
; the next start bit can arrive.  Uses r24, r25.
        .global rx
rx:
        sbic    VPORTA_IN, RX_BIT       ; wait for a falling edge
        rjmp    rx
        rcall   half_bit                ; to the middle of the start bit
        sbic    VPORTA_IN, RX_BIT
        rjmp    rx                      ; high again: a glitch, not a start bit
        ldi     r24, 0x80               ; falls out into C after the eighth bit
1:      rcall   half_bit
        rcall   half_bit
        lsr     r24
        sbic    VPORTA_IN, RX_BIT
        ori     r24, 0x80
        brcc    1b
2:      sbis    VPORTA_IN, RX_BIT       ; wait for the stop bit
        rjmp    2b
        ret

; Send r24.  The byte goes out inverted from C: com sets C for the start bit,
; and lsr shifts in the 0s that become the stop bit.  Uses r23, r24, r25.
        .global tx
tx:
        com     r24
        ldi     r23, 10                 ; start bit, 8 data bits, stop bit
1:      brcc    2f
        cbi     VPORTA_OUT, TX_BIT
        rjmp    3f
2:      sbi     VPORTA_OUT, TX_BIT
3:      rcall   half_bit
        rcall   half_bit
        lsr     r24
        dec     r23
        brne    1b
        ret
