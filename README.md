# RC5 receiver
## Introduction

This ESP-IDF component is designed for receiving RC5 infrared remote control signals. It includes an algorithm for handling auto-repeat functionality, which can be enabled or disabled based on the application's requirements.

### Callback Function

The prototype of the callback function is as follows:

```c
void rc5_handler(rc5_data_t rc5_data);
```


### Features
- Initialization and configuration of the RC5 receiver
- Decoding of RC5 signals
- Handling of received commands
- Auto-repeat function

### Usage
1. Initialize the RC5 receiver by calling the appropriate initialization function and passing a pointer to a callback function.
2. The callback function will be called when a valid RC5 packet is received.
3. Process the decoded commands within the callback function as needed in your application.

### Example
```c
// Callback function to process received RC5 commands
void rc5_handler(rc5_data_t rc5_data)
{
    // Process the command (send messages, generate events etc)
    process_rc5_command(rc5_data);
}

void app_main(void)
{
    // Initialize the RC5 receiver with the callback function
    rc5_receiver_init(rc5_handler);

    // Main loop
    while (true) {
        // Application code
    }
}
```

### Auto-Repeat Algorithm

When the auto-repeat algorithm is enabled, the component forwards every n-th received RC5 packet to the `rc5_handler` callback function. The value of `n` is determined by the `rc5_auto_repeat_postscaler` parameter. This allows the application to handle repeated commands at a controlled rate, preventing excessive processing of repeated signals.

If the auto-repeat algorithm is disabled, only a single command is sent to the `rc5_handler` callback function when a key is pressed. No additional commands are sent until the current key is released and a new key is pressed.

## Public interface

### Public Interface

The following functions are available in the RC5 receiver component:

#### `void set_auto_repeat(bool enabled, int threshold)`

Enable or disable auto-repeat functionality.

- **Description**: This function sets the auto-repeat feature for the component. When enabled, the component will automatically repeat actions after a specified threshold.
- **Parameters**:
    - `enabled`: A boolean value to enable (`true`) or disable (`false`) auto-repeat.
    - `threshold`: An integer value specifying the threshold for auto-repeat.

#### `esp_err_t rc5_receiver_init(rc5_handler_t rc5_handler)`

Initialize the RC5 receiver.

- **Description**: This function initializes the RC5 receiver with the provided handler. It sets up the necessary configurations and prepares the receiver for operation.
- **Parameters**:
    - `rc5_handler`: A handler function of type `rc5_handler_t` to process received RC5 signals.
- **Returns**: `ESP_OK` on successful initialization, or an error code on failure.

#### `void rc5_terminate(void)`

Terminate the RC5 receiver.

- **Description**: This function terminates the RC5 receiver, releasing any resources that were allocated during initialization. It should be called when the receiver is no longer needed.

## Implementation details.

### Implementation Details

The RC5 receiver component is implemented in the `rc5_receiver.c` file. Below are the key implementation details:

#### Initialization

The `rc5_receiver_init` function initializes the RC5 receiver by configuring the necessary hardware peripherals and setting up the interrupt service routine (ISR) to handle incoming RC5 signals. It also registers the provided callback function to process the decoded RC5 commands.

#### Signal receiving

The input signals (rmt symbols) are received in `rmt_symbol_word_t rc5_buffer[RC5_BUFFER_SIZE]`. After a RC5 packet si received, `rmt_rx_done_callback` is called. It copied received symbols from `rmt_symbol_word_t rc5_buffer[RC5_BUFFER_SIZE]`to `rmt_symbol_word_t rc5_buffer_cp[RC5_BUFFER_SIZE]`. Then it notifies the `rc5_receive_task` thread, which performs the actual decoding by calling the `rc5_decoder` function. Once decoding is complete, the decoded command is passed to the registered callback function. After this notification, `rmt_rx_done_callback` calls ` rmt_receive` so as to initiate new receiver session.

#### Auto-Repeat Handling

The auto-repeat functionality is managed by a postscaler counter within the `rc5_receive_task` thread. When auto-repeat is enabled, the counter increments with each received RC5 packet. Once the counter reaches the specified threshold, the decoded command is forwarded to the callback function, and the counter is reset. This ensures that repeated commands are processed at a controlled rate.

#### Termination

The `rc5_terminate` function stops the RC5 receiver by disabling the hardware peripherals and unregistering the ISR. It also releases any resources that were allocated during initialization.

#### Error Handling

The component includes error handling mechanisms to ensure robust operation. If an error occurs during initialization or signal decoding, appropriate error codes are returned, and the component attempts to recover or notify the application of the failure.

These implementation details ensure that the RC5 receiver component operates efficiently and reliably, providing accurate decoding of RC5 remote control signals and flexible handling of auto-repeat functionality.


