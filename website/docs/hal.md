## Introduction to the HAL layer
In this section we will explain the Hardware Abstraction Layer. This section is the one responsible, as the name say, to abstract the hardware. Hardware, when doing software design, is understood as all the interaction of the software with peripherals. All peripherals are mostly hardware registers in the microprocessor.
Say for example, you want to send an UART message. You can define a function like:

```
void UART_transmit(void *p_message);
```

When you implement this function, depending on the board or microcontroller used, it would mean to write data to some hardware registers, that will execute your request. For each board, these registers vary, and this is what differs from one platform to the other. This is why, when calling the function, in the real implementation, we call a pointer. 

```
void UART_transmit(void *p_message){
	if (p_message != NULL){
		pointer_to_function->sendMessage(p_data);
	}
}
```

This pointer has to be defined when starting the program execution. We mean that you need to define in the main function or somewhere, where does the real implemetation of this function take place (in which memory address will the processor find the lines of assembly code that has the implementation of how to write to the UART registers to send a message). In this example, you need to tell __pointer_to_function->sendMessage()__ where is this function implementation located.
This is good, because now and from this point on, if you develop above this, any service you use, it will call this abstracted function, and only when linking the abstract <-> real implementation, then it does something. This is the main design of the Solaris HAL. For example, you can create a service that calls this UART_transmit function, but it does not care how it is implemented. The service does not care if the UART is for one board or another. It may happen that you forget to tell the pointer, and the function points to NULL, causing and undefined behaviour. This is the responsability of the developer.


## The Solaris HAL layer
Once we have made a quick introduction to the Solaris HAL layer, we can now explain all of it parts, and how have we organized it. We will visit file by file in the current Solaris Packet Protocol.
Everything related to what will be explain, you will find it in the /hal folder of the SPP repo.

### hal.h 
In the hal.h file, we have defined a struct that contains multiple structs. This is:

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
This struct contains all the function pointer to all the HAL functions. This is what you need to initialize when starting your code. For each of the functions on this struct, you need to indicate where the real implementation is located. For example we can take a look at the UART struct:

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

As we can see, we have another struct that defines the function pointer signatures for initializing the UART port and for transmitting data via the UART port. We normally define where ths points to in the /ports/hal/esp32s3/halEsp32S3.c directory, for each board. For example, for the ESP32S3 board, we have something like this:

```
const static SPP_HALUart_t s_esp32HalUart = {.uartPortInit = SPP_PORTS_HAL_ESP32_uartPortInit,
                                             .uartTransmit = SPP_PORTS_HAL_ESP32_uartTransmit};
```

As you can see, here we are saying that each time we call the pointer *uartTransmit, we are poiting to SPP_PORTS_HAL_ESP32_uartTransmit this function, which has the real implementation. 
This allows the service to call the function pointer and we can switch as we wish to where does the function point to.

## Flux of calling the hardware fuctions
This is all nice, but you might be wondering how is the flux to call a hardware function.

