The Publish-Subscribe (PUBSUB) service coordinates the system. It registers producers and assigns them an APID, and it registers consumers, subscribing each of them to the producers they are interested in.

## Producers

A producer is an SPP service responsible for producing data. This can be a service backed by external hardware — the barometer service producing pressure and temperature readings, for example — or a purely software-based one, such as the log service. Anything that generates data, whether from a hardware peripheral or from a log format, falls into this category.

Like every service, a producer must comply with a contract: a predefined set of functionality it needs to implement so that the PUBSUB service can treat it as a producer. The contract is defined in services.h:

```
typedef struct
{
    const char *p_nameProducer;    /**< Human-readable module name (for logging). */
    spp_uint16_t timeoutMs;        /**< Timeout for the producer to send a packet */
    SPP_RetVal_t (*init)();        /**< Initialise the module. */
    SPP_RetVal_t (*acquireData)(); /**< Acquire data from the module. */
} SPP_SERVICE_ProducerContract_t;
```

When defining a producer, fill in this struct with:

- Name of the producer.
- Timeout: this is still pending functionality, but the plan is for the PUBSUB service to use it to stop giving CPU time to a producer that dies or malfunctions.
- init: a pointer to the producer's initialization function. The PUBSUB service calls it for every registered producer during system startup. This is normally where all producer-specific setup happens — for the barometer, that might mean talking to the sensor over the communication interface to wake it up, configure its registers, and put it in an operational state before the control loop starts.
- acquireData: a pointer to the function responsible for acquiring the producer's data and publishing it so that consumers can consume it. In practice, this is a private function inside the producer that talks to the hardware over the communication interface (SPI, I2C, UART, ...) to read its registers. Once the data has been read, the producer asks the DATABANK for an empty packet, hands it the data to pack, and publishes the result to the PUBSUB service:

```
SPP_SERVICES_DATABANK_getPacket()
SPP_SERVICES_DATABANK_packetData()
SPP_SERVICES_PUBSUB_publish()
```

From this point on, delivering the packet is the PUBSUB service's responsibility, not the producer's. The PUBSUB service delivers a copy of the packet to every consumer subscribed to it — via the mailbox, described below — and then returns the original packet to the DATABANK so it can be emptied and made available for reuse. If no consumer is subscribed to that data, the PUBSUB service returns the packet to the DATABANK straight away.

## Consumers

A consumer is a service that consumes data from producers. Once a producer has published an SPP packet, it's the subscribed consumers that pick it up and act on it — saving it to an SD card, saving it to eMMC, sending it over an antenna, and so on, depending on what your project needs.

Consumers also follow a contract:

```
typedef struct
{
    spp_int8_t priority;         /**< Priority of the consumer */
    const char *p_nameConsumer;  /**< Human-readable module name (for logging). */
    spp_uint16_t timeoutMs;      /**< Timeout for the consumer to receive a packet */
    spp_uint16_t suscribeToApid; /**< APID subscription bitmask */
    SPP_RetVal_t (*init)();      /**< Initialise the module. */
    SPP_RetVal_t (*deliverToMailbox)(const SPP_Packet_t packet);
    SPP_RetVal_t (*consumeData)(void *p_data); /**< Consume data from the module. */
} SPP_SERVICE_ConsumerContract_t;
```

- Priority: the higher the value, the more priority it has. This is what lets the PUBSUB service order the registered consumers, so that the ones with the most priority run first.
- Name of the consumer.
- Timeout: same idea as for the producer — if a consumer takes too long, some measure has to be taken to stop giving it CPU time.
- subscribeToApid: set when registering the consumer with the PUBSUB service, it determines which producer's data gets sent to this consumer's mailbox.
- init: same concept as before, a pointer to a private function responsible for initializing the consumer.
- deliverToMailbox: a pointer to a function that receives a copy of an SPP packet. Think of it as the delivery guy having a letter for you: he puts it in your mailbox so you can check its contents later. The only responsibility here is to take that packet copy and store it in a static array, nothing more. This step is delicate, because if it doesn't return, it blocks the rest of the program too — that is why we might revisit this API in the future to make it less error-prone. You can look at the existing consumers for reference.
- consumeData: a pointer to the function responsible for actually doing something with that data. Here you cannot block the execution of the system. You can't poll on things, for example, because right now there's no way to stop a service from running — everything runs bare-metal, on a single core. If a service takes too long, it affects interrupts, data acquisition, and every other service, which is why consumers need to be fast and non-blocking.

## Flow of information

This can be a bit overwhelming, so let's go through how the PUBSUB service works internally.

![PUBSUB inner workings](assets/pubsub.svg){ .center }

Based on the figure above, there are three distinct stages. The first is when consumers and producers are registered with the PUBSUB service. This is custom, defined by the team or user building the binary — you choose how many producers and consumers your project has, and which consumers subscribe to which producers. In our FSM (Finite State Machine), this happens in the init state, inside a private function called __SPP_CORE_FSM_registerConsumerProducer__: just before anything else runs, the user registers the services it wants as producers and consumers. A quick look at that function in fsm.c should make it clear; the FSM is covered in more detail in the next chapter. This is the top part of the figure.

The second stage happens when __PUBSUB_init__ is called. It walks over all the registered consumers and producers and calls the init function pointer from each one's contract. It first sorts the consumers by priority, then initializes every module in that order — that's the middle part of the figure.

Lastly, there's the ongoing work of the PUBSUB service, shown in the bottom part of the figure. The PUBSUB service calls the acquireData function of every registered producer. If a producer has data, it publishes it through __publish__, as described above. The PUBSUB service then checks whether any data has been published and, if so, delivers a copy of it to the subscribed consumers. Once a copy has been delivered, the packet is returned to the DATABANK service so it can be cleared and made available again. Finally, the PUBSUB service calls the __consumeData__ function of every registered consumer, in the same priority order it sorted them into back in __PUBSUB_init__.

That's it — the PUBSUB service is mainly responsible for managing SPP packets and delivering them to the right parts of the system in a safe way. Next, we'll look at the Finite State Machine, the last piece needed to understand how the full system fits together.
