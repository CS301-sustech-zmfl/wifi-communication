# Wi-Fi Project Report--SUSTech-STM32-EPS8266

## Introduction

This report is for course CS301(*Embedded System and Microcomputer Principle*) term project of Wi-Fi. ATK-ESP8266 is a UART-Wi-Fi module with high performance. This project uses ATK-ESP8266 module to realize the wireless communication between MCU. We establish communication between two ESP8266 Wi-Fi module based on MiniSTM32 board.

##  Requirements and functionalities

- Establish default connection when pressing key0, and close default connection when pressing key1. 
- Through the serial debugging assistant as the connection and other configuration command input mode, input specific command to configure and operate related connection properties, can establish or close the connection.
- Show connection and communication status through led lights as the signals. 
- Communicate with each other by serial debugging assistant. 
- Connection detection automatically, and give tips under the circumstances of long distances, inferences or shutting down.
- To show the contents of chatting message on the LCD screen and information about the connection status (right or wrong), simple but good-looking UI design.
- Design the function operation rationally.

## System architecture design

### Design Ideas

According to the requirements, we separate the system functionalities into xx functions.

- **Reset**: When pressing the reset key, then we need to restart the MiniSTM32 board and Wi-Fi module.
- **Establish connection function**: The connection state of 2 MCU is established by sending and receiving the connection message. Click the KEY0 to restart the Wi-Fi module and send the connecting message to server when it acts as a client, and set the default AP settings when it acts as a server.
- **Check connection function**: The function is to test whether the connection is still alive without sending or receiving messages. I use the TIM3 interruption to send the heartbeat command to check whether the connection is still alive, if it is not alive, then enter the idle state.
- **Communication function**: The function is used to send messages to the other side or receive messages from the other side. Each message transmission will define the link number and the length of the message to be sent.
- **Display function**: The function is to display the messages sent and received, the connection status on the LCD screen.

The picture below is the **FSM** of our project.

<img src=".\pictures\image-20201220121028173.png" alt="image-20201220121028173" style="zoom:67%;" />

### Pinout view

The picture below is the pinout view about our project.

<img src=".\pictures\image-20201219211430366.png" alt="image-20201219211430366" style="zoom:67%;" />

We set up the `USART1, USART2, GPIO, TIM3, LCD` as the project configuration.

`USART1`: This serial port is to receive the input from the user either to control the status of Wi-Fi or send message to the other part.

`USART2`: This serial port is to receive the messages from the Wi-Fi module and send the command to the Wi-Fi module.

`TIM3`: This clock is used to generate periodic interruption, which can be used in the `check connection function`.

`LCD`: LCD is used to display the necessary information such as system status, communication messages, etc., on the LCD screen.

## Code implementation

### GPIO callback function

This function is to establish the default connection, according to the eps8266 mode.

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    HAL_Delay(100);
    if (esp8266 is a server) {
        // for the server mode
        switch (GPIO_Pin) {
            case KEY0_Pin:
                // establish the connection
                if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
                    init_server();
                }
                break;
            case KEY1_Pin: 
                // to disconnect
                if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                    close_server();
                }
                break;
            default:
                break;
        }
    } else {
        //for the client mode
        switch (GPIO_Pin) {
            case KEY0_Pin:
                if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
                    init_var();
					//TODO: SET THE CLIENT MODE
                    HAL_Delay(10000);//WAIT FOR CONNECTION
                    int i = 0;
                    while (connect_flag != 2 && i <= 3) {
                        //attempt to join the ap for 3 times 
                        connect_flag = 1;
                        if (i >= 3) {
                            send_msg_uart1(conn_fail_msg, 0);
                            send_msg_uart1(split, 0);
                            HERE: change the LED STATUS
                            connect_flag = 0;
                            return;
                        }
                        send_cmd(join_ap, 10000);
                        ++i;
                    }
                    //TODO: SET THE CLIENT MODE
                    break;
                }
            case KEY1_Pin:
                if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                    close_connection();
                    break;
                }
            default:
                break;
        }
    }
}
```

### TIM3 callback function

This function is used to` check the connection`between the server and the client every 3 seconds when there is no sending or receiving messages.

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == (&htim3)) {
        ++timer_count;
        timer_count %= 15;
        if (timer_count == 0 && connect_flag == 2) {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            if (esp8266_mode == 1) {
                //server mode
                send_cmd((uint8_t *) "AT+PING=\"192.168.4.x\"\r\n", 0);
                //x is to be defined
            } else {
                // to make ping to 192.168.4.1 (default gateway)
                send_cmd((uint8_t *) "AT+PING=\"192.168.4.1\"\r\n", 0);
            }

        }
        if (connect_flag == 0) {
            // there is no connection
            HERE: change the LED STATUS
            return;
        }
        if (connect_flag == 1) {
            // It is connecting
            HERE: change the LED STATUS
            return;
        }
        if (connect_flag == 2) {
            if (sending_flag == 0) {
                // It is connected and there are no sending messages
            HERE: change the LED STATUS
            } else if (sending_flag == 1) {
                // It is connected and there are sending messages
            HERE: change the LED STATUS
            }
            return;
        }
    }
}
```

### UART callback function

This function is used to receive and forward the messages to the destination. When receiving message from the user input, it forwards it to the Wi-Fi module and maybe further sending the message to the other Wi-Fi module, and when receiving the message from the Wi-Fi module, it forwards to the USART1 serial port and whether the message will be displayed depends on the codes.

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // to receive the data from the usart1
    if (huart->Instance == USART1) {
        if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);
            HAL_UART_DMAStop(&huart1);
            uint8_t data_length = 2048 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
            if (link_number != -1 && uart1_rx_buffer[0] == '>' && connect_flag == 2) {
                //Here: set the light on and off according to the state 
                    //set the sending flag to 1
                //send_message(uart1_rx_buffer + 1);
 				//Here: set the light on and off according to the state 
                    //set the sending flag to 0
                return;
            }
            if (strcmp(uart1_rx_buffer, server_mode) == 0) {
                esp8266_mode = 1;
                send_msg_uart1((uint8_t *) "SET AS SERVER\r\n", 0);
                send_cmd(ap_sta_cmd, 1000);
                return;
            }
            if (strcmp(uart1_rx_buffer, client_mode) == 0) {
                esp8266_mode = 0;
                send_msg_uart1((uint8_t *) "SET AS CLIENT\r\n", 0);
                send_cmd(sta_cmd, 1000);
                return;
            }
            if (strncmp(uart1_rx_buffer, "AT+CWQAP",8) == 0 && esp8266_mode == 0) {
                HAL_UART_Transmit(&huart2, (uint8_t *) uart1_rx_buffer, data_length,
                                  0xffff);
                init_var();
                return;
            }

            HAL_UART_Transmit(&huart2, (uint8_t *) uart1_rx_buffer, data_length,
                              0xffff);
            memset(uart1_rx_buffer, 0, data_length);
            data_length = 0;
            HAL_UART_Receive_DMA(&huart1, (uint8_t *) uart1_rx_buffer, 2048);
        }
    } else if (huart->Instance == USART2) { 
        // to receive the data from the usart2
        if (RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);
            HAL_UART_DMAStop(&huart2);
            uint8_t data_length = 2048 - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

            if (strncmp(uart2_rx_buffer, "WIFI GOT IP\r\n", 13) == 0) {
                Here: set the light on and off according to the state
                connect_flag = 2;
                link_number = 0;
                return;
            }
            //client 端会收到这个消息
            if (connect_flag == 2) {
                if (strcmp(uart2_rx_buffer, "WIFI DISCONNECT\r\n", 16) == 0) {
                    init_var();
                    HAL_UART_Transmit(&huart1, (uint8_t *) uart2_rx_buffer, data_length,
                                      0xffff);
                    return;
                }
                if (strncmp(uart2_rx_buffer, rx_header, 7) == 0) {
                    //收到来自另一端的消息
                    HAL_UART_Transmit(&huart1, (uint8_t *) uart2_rx_buffer, data_length,
                                      0xffff);
                    return;
                }
                if (strncmp(uart2_rx_buffer, (uint8_t *) "0,CLOSE OK", 10) == 0) {
                    init_var();
                    HAL_UART_Transmit(&huart1, (uint8_t *) uart2_rx_buffer, data_length,
                                      0xffff);
                    return;
                }
                // check the connection
                if (strncmp(uart2_rx_buffer, (uint8_t *) "+", 1) == 0) {
                    if (strstr(uart2_rx_buffer, "+timeout") != NULL) {
                        if (connection_counter > 2) {
                            send_msg_uart1((uint8_t *) "connection closed\r\n", 100);
                            send_cmd((uint8_t *) "AT+CIPCLOSE=0\r\n", 2000);
                            init_var();
                            connection_counter = 0;
                        } else {
                            send_msg_uart1((uint8_t *) "connection disturbance\r\n", 100);
                            ++connection_counter;
                        }
                    } else {
                        connection_counter = 0;
                        send_msg_uart1((uint8_t *) "connection alive\r\n", 100);
                    }
                    return;
                }

            }

            if (strncmp(uart2_rx_buffer, "0,CONNECT\r\n", 11) == 0) {
                link_number = 0;
                connect_flag = 2;
                HAL_UART_Transmit(&huart1, (uint8_t *) "Connection on id 0\r\n", 20, 0xffff);
                return;
            }
            if (strncmp(uart2_rx_buffer, "0,CONNECT FAIL\r\n", 16) == 0) {
                init_var();
                return;
            }
            if (strncmp(uart2_rx_buffer, "0,CLOSED\r\n", 10) == 0) {
                init_var();
                return;
            }
            memset(uart2_rx_buffer, 0, data_length);
            data_length = 0;
            HAL_UART_Receive_DMA(&huart2, (uint8_t *) uart2_rx_buffer, 2048);
        }
    }
}
```

### User defined function

These functions are used inside the functions above.

```c
void send_cmd(uint8_t *cmd, int delay_time) {
    HAL_UART_Transmit(&huart2, (uint8_t *) cmd, strlen(cmd), 0xffff);
    HAL_Delay(delay_time);
}

void send_message(uint8_t *msg) {
    // to send the message
}

void send_msg_uart1(uint8_t *msg, int delay_time) {
    HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 0xffff);
    HAL_Delay(delay_time);
}

void init_var() {
    //to initialize the variables
    //to change the state
    //to change the LED
}

void init_server() {
    //to initialize the variables
    //to send the commands to initialize the server
}
```

### User defined variables

```c
int esp8266_mode = 1;// esp8266_mode = 1 -> server mode, esp8266_mode = 0 -> client mode
int connect_flag = 0;//connect_flag = 0 -> not connected, 1-> wifi-connected, 2-> tcp-connected
int sending_flag = 0;//set the sending flag, 1-> is sending, 0 -> is not sending
int link_number = -1;//linking number of the tcp
int timer_count = 0;
int connection_counter = 0;

uint8_t conn_fail_msg[] = "...Connecting fails...\r\n";
uint8_t *look_up_cmd = "AT+CIFSR\r\n";
/**--------------**/
uint8_t *set_ap = "AT+CWSAP=\"SUSTC\",\"sustcyyds\",1,0,4,0\r\n";
uint8_t *set_server = "AT+CIPSERVER=1,8089\r\n";
uint8_t *start_tcp = "AT+CIPSTART=0,\"TCP\",\"192.168.4.1\",8089\r\n";
/*--switching the mode--*/
uint8_t sta_cmd[] = "AT+CWMODE=1\r\n";//to be the station
uint8_t ap_sta_cmd[] = "AT+CWMODE=3\r\n";//to be both the ap and the station
/*--end--*/
uint8_t join_ap[] = "AT+CWJAP=\"SUSTC\",\"sustcyyds\"\r\n";
uint8_t quit_ap[] = "AT+CWQAP\r\n";
/*-------*/
uint8_t set_mux[] = "AT+CIPMUX=1\r\n";
/*-------*/
uint8_t restart[] = "AT+RST\r\n";
uint8_t at[] = "AT\r\n";
/*-------*/
uint8_t split[] = "||=========================||\r\n";
/*-------*/
uint8_t server_mode[] = "SERVER MODE\r\n";
uint8_t client_mode[] = "CLIENT MODE\r\n";
/*-------*/
uint8_t send_finish_msg[] = "\r\nSEND OK\r\n";
uint8_t sending_fail[] = "SEND FAIL\r\n";
uint8_t rx_header[8] = "\r\n+IPD,";
```

## Problems & solutions

1. Receiving `Link type ERROR` when typing the right command `AT+CIPSTART=0,"TCP","192.168.4.1",8089`.

   **SOLUTION:** After joining the AP, we should wait for 5 seconds or more, and then to establish the TCP connection. Solution found via [CSDN](https://blog.csdn.net/qq_34020487/article/details/100904978).

2. **This problem is provided and solved by XXX**. When we receive a message using `HAL_UART_Receive_IT`, if there are multiple carriage returns (`\n`) in the message, a portion of the message after the first carriage return
   will be lost.

   **SOLUTION:** Change the serial port to `DMA` mode and use `HAL_UART_Receive_DMA` to receive messages.

3. The priority problems about the multiple interruptions.

   **SOLUTION:** We found that the MiniSTM32 board does not have operating system supported, so there is no way manage threads or processes, then we simplify the operation, make the LED status changed when the system status changes.

## Results

This part is contained in the file `Wi-Fi_STM32_Manual.pdf`.

## Use Manual

The manual is attached as the file `Wi-Fi_STM32_Manual.pdf`.

## Summary

In this project we are introduced to the basic hardware coding world, which is fantasy but hard to debug. We establish the connection and communication between two MiniSTM32 board using the ESP8266 Wi-Fi modules. We further understand and grasp how to use the interruption functions and set the priority of the interruption, how to send and receive data through the USART, and display what we need on the LCD screen. What's more, we have deepened the understanding of the embedded programming.

## THE END--HAPPY HARDWARE CODING!