# General Layer Description
Solaris Software is built in layers. Each layer has no knowledge of the layers above or below it, so you can swap out or change what's inside a layer without breaking the rest of the code. The image below shows how they stack.

![Layer stack](assets/layers-1.svg){ .center }


## HAL layer
The lowest layer is the HAL, or Hardware Abstraction Layer. It's the layer responsible for talking to the board's hardware — SPI, UART, clocks, I2C, and so on. If you need to reach a peripheral, this is where you'll find the function for it.

The interesting part is that, because the HAL is the "abstracted" side of the code, it doesn't actually implement any functionality itself. For example, in the HAL layer you'll find a function like this one:

```
/**
 * @brief Perform a full-duplex SPI transaction.
 *
 * @param[in,out] p_handle  SPI device handle.
 * @param[in,out] p_data    TX data in, RX data out (in-place).
 * @param[in]     length    Number of bytes in the transaction.
 *
 * @return K_SPP_OK on success, K_SPP_ERROR_ON_SPI_TRANSACTION on failure.
 */
SPP_RetVal_t SPP_HAL_SPI_transmit(void *p_handle, spp_uint8_t *p_data, spp_uint8_t length);
```

This function sends an SPI transaction, and lives at solaris-packet-protocol/hal/spi/spi.h. But look at its implementation and you'll be a little underwhelmed — it looks like this:

```
SPP_RetVal_t SPP_HAL_SPI_transmit(void *p_handle, spp_uint8_t *p_data, spp_uint8_t length)
{
    SPP_RetVal_t ret = K_SPP_ERROR;

    const SPP_HalPort_t *p_port = SPP_HAL_getPort();
    if ((p_port == NULL) || (p_port->spi.spiTransmit == NULL))
    {
        ret = K_SPP_ERROR_NULL_POINTER;
    }
    else
    {
        ret = p_port->spi.spiTransmit(p_handle, p_data, length);
        if (ret != K_SPP_OK)
        {
            ret = K_SPP_ERROR;
        }
    }
    return ret;
}
```

Notice that no board-specific code is called directly — instead, it goes through a function pointer, __p_port->spi.spiTransmit(p_handle, p_data, length)__. That's what makes the abstraction work: the HAL itself stays independent of the platform. We go into how this gets set up and modified in the [HAL section](hal.md).

## SPP Core layer
Above the HAL sits the SPP Core layer, which holds the basic functionality the SPP needs to run — mandatory services such as the databank, packet generation, CRC, and the pub/sub service. We'll go through each of these in later sections. The core is always included in the final binary.

## Services layer
Above the core sits the services layer, which we like to think of as the "app store". Developers write and validate independent modules, then plug them into the core to make them run. SPP has two kinds of services — consumers of data and producers of data — and both follow a "contract" that we'll detail later. You can add as many services as you like, but each one adds to the control cycle time. A rocket usually doesn't need many services, but on a bigger system this is worth keeping in mind: the more services you add, the longer your control cycle takes.

## Driver implementations
Last come the drivers — the platform-specific part, where the function pointers we saw earlier finally get implemented. This is the real code that ends up being called through those pointers.

You write one driver implementation per board SDK (Software Development Kit). For each platform, the drivers need to be developed to match the SPP function signatures defined in the HAL. It's a bit of extra work, but not much, and once it's done you get to reuse the rest of the software as-is. Right now we're adding support for both the ESP32S3 and the RP2350, so you'll find implementations for both — feel free to use them as a reference. And if you run into a case where our abstraction doesn't quite fit and something's missing, let us know and we'll fix it right away.


And that's the gist of it. There's more nuance to it, of course, which is why the rest of the documentation goes deeper, but this should give you the main idea. One thing worth pointing out: there's no OSAL (Operating System Abstraction Layer) here — Solaris runs bare-metal, for several reasons, one of the biggest being control over execution.
