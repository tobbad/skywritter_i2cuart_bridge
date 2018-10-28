/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Tobias Badertscher
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * serial_io.h
 *
 *  Created on: 12.05.2018
 *      Author: badi
 */

#ifndef INC_SERIAL_IO_H_
#define INC_SERIAL_IO_H_

#define UART_TIMEOUT_MS 100

typedef enum {SIO_ERROR=-1, SIO_OK=0, } sio_res_e;
typedef enum {SIO_RX=0, SIO_TX, SIO_RXTX_CNT} sio_channel_e;

typedef struct _sio_t
{
    UART_HandleTypeDef *uart;
    bool     ready[SIO_RXTX_CNT];   /* Internal use only */
    uint16_t buffer_size[SIO_RXTX_CNT];
    int16_t  bytes_in_buffer[SIO_RXTX_CNT];
    uint8_t *buffer[SIO_RXTX_CNT];

} sio_t;


sio_res_e serial_io_init(sio_t *init);

#endif /* INC_SERIAL_IO_H_ */
