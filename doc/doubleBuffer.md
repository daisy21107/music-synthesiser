# Embedded Systems Coursework 2
## Generating audio samples with a double buffer

An audio synthesiser needs to generate a new sample every $1/f_s$ seconds.
The sample frequency is much higher than an RTOS scheduler tick rate, so the lab notes use an interrupt which is triggered off a timer with a period of $1/f_s$.

Generating an audio sample can be computationally expensive, especially if convolution is used to implement a filter, or if several waveforms are being summed together.
Doing this computation on demand, once per sample period causes a large proportion of the CPU workload to have a high priority.
A large ISR workload can cause problems, such as:

- Need to set interrupt priorities carefully so that other interrupts complete on time.
- There can be a conflict with the RTOS, particularly if you need a higher priority than the interrupt used for the RTOS tick.
- Unavailability of blocking synchronisation functions (e.g. lock a mutex) makes synchronisation more difficult.
  The result is often a proliferation of critical sections in thread functions, which are a very coarse method of synchronisation.
  
A common solution is to calculate samples in batches in a lower-priority thread and use the interrupt just to copy the sample to the DAC.
You can also omit the interrupt entirely and set up the DMA to copy samples - the DMA has auto-increment and timer features for exactly this kind of application.

Whether the copy to DAC is done by interrupt or DMA, there is an intermediate storage of samples in memory that is shared between two tasks: sample generation and copy to DAC.
That means it requires synchronisation.

### Double Buffer

A common method of synchronising this kind of transfer is to use a double buffer, or ping-pong buffer.
A double buffer is simpler and more efficient than a queue, but it can be written by only one task and read by only one task.

The principle is to split an array into two.
The writing task will access one half and the reading task will access the other.
When both tasks have finished accessing half the buffer, they will swap pointers and access the other half.

Synchronisation is achieved by ensuring that the pointer swap happens atomically, so the two tasks never access the same half of the buffer.

### Coursework Implementation with ISR

Create two arrays that will hold the samples, with a flag that will define which half is being written:

```c++
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE/2];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE/2];
volatile bool writeBuffer1 = false;
```

The ISR will copy one sample to the DAC and increment a write counter.
The flag is used to determine which buffer to read from.
When the pointer reaches `SAMPLE_BUFFER_SIZE/2`, the counter will reset to zero and the pointers will swap:

```c++
static uint32_t readCtr = 0;

if (readCtr == SAMPLE_BUFFER_SIZE/2) {
	readCtr = 0;
	writeBuffer1 = !writeBuffer1;
	xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
	}
	
if (writeBuffer1)
	analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
else
	analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
```

A sempaphore is given by the ISR when the buffer swaps.
You'll need to declare a pointer for it and initialise it on startup.

```c++
sampleBufferSemaphore = xSemaphoreCreateBinary();
xSemaphoreGive(sampleBufferSemaphore);
```

That semaphore will be used as the blocking statement in a sample generation thread that fills one buffer with samples in a single batch.
The main loop of that thread will look something like this:

```c++
while(1){
	xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
	for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE/2; writeCtr++) {
		uint32_t Vout = … //Calculate one sample
		if (writeBuffer1)
			sampleBuffer1[writeCtr] = Vout + 128;
		else
			sampleBuffer0[writeCtr] = Vout + 128;
	}
}
```

The semaphore is given when it is created so that the thread doesn't block on its first loop and the first batch of samples is generated straight away.
You may want to initialise the sample buffers with midpoint values (e.g. 128) to send to the output while the first batch is being generated.
	
### Buffer size
The size of the sample buffer affects the priority of the sample generation thread and the latency between generating a sample and it appearing on the output.
One batch of samples must be generated for every $n$ samples, where $n$ is the size of each buffer.
If samples are consumed at $f_s$, then sample generation has an initiation interval of $n/f_s$ seconds, and the priority of the thread must be set accordingly.

Meanwhile, the latency $L$ between generation and output depends on the time it takes to generate a batch of samples.
$L \gt n/f_s$, since every sample must wait for a pointer swap until it is output.
The maximum latency depends on the rate of sample generation, which will be faster than sample consumption until a batch is complete and the generator thread waits for a pointer swap.
If sample generation is very fast, then the last sample in a batch could be generated soon after a pointer swap and sent to the output at the end of the next cycle.
So $n/f_s\lt L \lt 2n/f_s$ seconds.
If $L$ is too large then there could be a noticable delay between a user input and the resulting sound.

### Limitations and assumptions
The double bufffer implementation here is lightweight but it makes some assumptions:
1.	The ISR makes several access to global memory: the sample buffer, the `writeBuffer1` flag and the semaphore.
	There is no attempt to keep them synchronised and we assume that there is no higher-priority task that could prempt this interrupt and access these variables.
	For example, there is no possibility of the write thread acting on the change to `writeBuffer1` before the semaphore is given in the next line.
2.	The `writeBuffer1` flag is atomic, but the generator thread uses it non-atomically to decide which buffer to write to.
	There is a possibility of the flag changing between the test and the write to a sample buffer, so the flag doesn't fully protect against simultaneous access to the same buffer.
	This could be resolved by placing the flag test and the buffer write in a critical section.
	
	However, we do not expect the flag to change during a batch of sample generation because the flag changes are synchroised with the initiation intervals of the generator task.
	If it does change, that means generation cannot keep up with the sample rate — any synchronisation failure here would be a side-effect of a deadline miss.
	The code could be improved by testing for this condition and indicating an exception in some way.
3. 	The code uses C arrays, which have no built-in protection against out-of-bounds access.
	We assume there are no bugs that would result in accessing data outside an array.

An alternative is the [FreeRTOS Stream Buffer](https://www.freertos.org/RTOS-stream-buffer-example.html).
A stream buffer has a lower overhead than a queue and also has the limitation of one reader and one writer.
However, it would not be suitable for use with DMA without modification because the DMA would need to access its internal storage directly.
	
### Double buffering with DMA
The STM32 DMA module is designed to support this scenario: streaming samples from a double buffer in RAM to the DAC, but it will take some research to understand how to configure and use it. The main resources are the [STM32L4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) (section 11) and the [STM32L4 HAL User Manual](https://www.st.com/resource/en/user_manual/um1884-description-of-stm32l4l4-hal-and-lowlayer-drivers-stmicroelectronics.pdf) (sections 16 and 22). You can also use STM32CubeMX to generate configuration code based on settings in a GUI.

At a high level, the steps are:
1. Configure the DAC to be triggered by a timer
2. Configure the timer to have a period equal to $1/f_s$
3. Configure the DMA for memory-to-peripheral copy, with DMA requests from the DAC
4. Write an ISR that will trigger when the DMA transfer is complete. It will start the next transfer and set the semaphore to trigger the next batch of sample generation. Alternatively, you can run the DMA in cicrcular mode where it will run continuously and trigger interrupts when it is complete and half complete. Register the ISR as a callback with the HAL
5. Start the first DMA transfer, specifying the addresses and size of the transfer (`HAL_DAC_Start_DMA()`)

