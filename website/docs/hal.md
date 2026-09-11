## What a HAL is, and why it uses function pointers
This section explains the Hardware Abstraction Layer, or HAL. As the name says, its job is to abstract the hardware. In software design, "hardware" means anything the software talks to through peripherals, and peripherals are, in the end, just registers on the microprocessor.

Take UART. Solaris exposes it to the rest of the stack through a single function:

```
SPP_RetVal_t SPP_HAL_UART_transmit(const void *p_data, spp_uint16_t len);
```

Sending data over UART means writing to hardware registers, and those registers change depending on the board or microcontroller — that's exactly what differs from one platform to another. Because of that, SPP_HAL_UART_transmit doesn't touch any register itself. It looks up the active HAL port and calls through a function pointer instead:

```
SPP_RetVal_t SPP_HAL_UART_transmit(const void *p_data, spp_uint16_t len)
{
    const SPP_HalPort_t *p_port = SPP_HAL_getPort();

    if (p_port == NULL || p_data == NULL)
    {
        return K_SPP_ERROR_NULL_POINTER;
    }

    if (p_port->uart.uartTransmit == NULL)
    {
        return K_SPP_ERROR;
    }

    return p_port->uart.uartTransmit(p_data, len);
}
```

`p_port->uart.uartTransmit` is that function pointer, and it has to be pointed at the board's real implementation before any of this can work — at which memory address the processor will find the actual code that writes to the UART registers. That wiring happens once, at startup, when the board's HAL port is registered.

This is the whole point of the design: from here on, any service that wants to send something over UART just calls SPP_HAL_UART_transmit. It doesn't need to know how the UART is implemented, or which board it's running on — that's entirely between the pointer and whatever it was pointed at. And because the function checks the pointer before calling it, forgetting to wire it up gives you a clean K_SPP_ERROR instead of undefined behaviour.

## The hal.h struct hierarchy
Now that we've covered the idea, let's look at how it's actually organized in the SPP repo. hal.h defines the struct that groups together every HAL sub-struct — this is the SPP_HalPort_t that p_port pointed to above:

```
typedef struct
{
    SPP_HALSpi_t spi;
    SPP_HALGpio_t gpio;
    SPP_HALStorage_t storage;
    SPP_HALTime_t time;
    SPP_HALUart_t uart;
} SPP_HalPort_t;
```

Each field is itself a struct of function pointers, one per peripheral. Here's the UART one, which we've already seen in action:

```
/**
* @brief UART port struct that contains all the pointers to the UART functions
*/
typedef struct
{
    /**
     * @brief Initialise the UART port.
     *
     * @param[in] p_cfg Pointer to UART init configuration.
     *
     * @return K_SPP_OK on success.
     */
    SPP_RetVal_t (*uartPortInit)(void);

    /**
     * @brief Perform an UART transaction.
     *
     * @param[in,out] p_handle  SPI device handle.
     * @param[in,out] p_data    TX data in, RX data out.
     * @param[in]     length    Number of bytes in the transaction.
     *
     * @return K_SPP_OK on success, K_SPP_ERROR_ON_SPI_TRANSACTION on failure.
     */
    SPP_RetVal_t (*uartTransmit)(const void *p_data, spp_uint32_t len);

} SPP_HALUart_t;
```

Each board decides what these actually point to. For the ESP32S3, that's set in /ports/hal/esp32/halEsp32.c:

```
const static SPP_HALUart_t s_esp32HalUart = {.uartPortInit = SPP_PORTS_HAL_ESP32_uartPortInit,
                                             .uartTransmit = SPP_PORTS_HAL_ESP32_uartTransmit};
```

So every time something calls the pointer *uartTransmit, it's actually running SPP_PORTS_HAL_ESP32_uartTransmit, the real implementation.

The same pattern repeats one level up, for the board as a whole: each of the five HAL structs (spi, gpio, storage, time, uart) gets its own board-specific instance, and they're all grouped together into a single SPP_HalPort_t:

```
static const SPP_HalPort_t s_esp32HalPorts = {
    .spi = s_esp32HalSpi,
    .gpio = s_esp32HalGpio,
    .storage = s_esp32HalStorage,
    .time = s_esp32HalTime,
    .uart = s_esp32HalUart,
};
```

This is passed into the HAL layer once, at startup:

```
//Get HAl port
const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();
// Pass the HAL port to the FSM
(void)FSM_init((void *)p_halPorts);
```

We'll go into more detail on the FSM later, but for now just note that this struct is what ends up handed to the HAL layer, through:

```
/**
* @brief This functions initializes the ports struct and initializes all the peripherals.
*/
SPP_RetVal_t SPP_HAL_init(void *p_halPorts);
```

Once this is wired up, everything is abstracted: the same services work on any board without changes. This is what makes Solaris portable — you reuse the whole stack, and if you need something custom, you build it against the HAL layer instead of talking to the board drivers directly.

## Following a call through the HAL
That covers the pieces, but you're probably wondering what actually happens, step by step, when a hardware function gets called. The figure below shows the flow.

![HAL diagram](assets/hal.svg){ .center }

Let's imagine a service called "the messenger", responsible for sending logs over UART. To send its message, it calls SPP_HAL_UART_transmit, the same abstracted function from the very first example. That function looks up the active HAL port and calls p_port->uart.uartTransmit, the pointer from the SPP_HALUart_t struct above. On the ESP32S3, that pointer was set to SPP_PORTS_HAL_ESP32_uartTransmit, the code that actually writes to the UART registers.

So the call travels like this: the messenger calls SPP_HAL_UART_transmit (abstracted), which calls p_port->uart.uartTransmit (the pointer), which was set to SPP_PORTS_HAL_ESP32_uartTransmit (the real implementation) when the board's HAL port was built. The messenger never has to know which board it's running on, it just trusts that the pointer was wired up correctly during initialization.
