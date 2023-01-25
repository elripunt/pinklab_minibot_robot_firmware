#include <Encoder.h>
#include <ArduPID.h>

/*
9: PWM L_FORWARD
10: PWM L_BACKWARD
11: PWM R_FORWARD
12: PWM R_BACKWARD

18: L_ENC_A
19: L_ENC_B
20: R_ENC_A
21: R_ENC_B
*/

// Packet Data
uint8_t g_rx_buf[256] = {0, };
uint8_t g_recv_data[256] = {0, };
uint8_t g_rx_index = 0;

// Motor Related
uint8_t g_motor_enabled = 0;
uint8_t g_l_lamp_val = 0;
uint8_t g_r_lamp_val = 0;

int16_t g_l_motor_target = 0;
int16_t g_r_motor_target = 0;
int32_t g_l_current_encoder = 0;
int32_t g_r_current_encoder = 0;
int32_t g_l_last_encoder = 0;
int32_t g_r_last_encoder = 0;

// PID Related
double l_current_vel = 0;
double l_control_output = 0;
double l_target_vel = 0;

double r_current_vel = 0;
double r_control_output = 0;
double r_target_vel = 0;

double p_gain = 12;
double i_gain = 1;
double d_gain = 0.5;

ArduPID l_motor_controller;
ArduPID r_motor_controller;

int32_t old_l_enc = 0;
int32_t old_r_enc = 0;

Encoder l_enc(18, 19);
Encoder r_enc(20, 21);


void setup() {  
  g_motor_enabled = 0;
  g_l_lamp_val = 0;
  g_r_lamp_val = 0;

  g_l_motor_target = 0;
  g_r_motor_target = 0;
  g_l_current_encoder = 0;
  g_r_current_encoder = 0;
  g_l_last_encoder = 0;
  g_r_last_encoder = 0;

  l_enc.write(0);
  r_enc.write(0);

  // Timer: Control loop
  cli();
  TCCR4A = 0; // set entire TCCR4A register to 0
  TCCR4B = 0; // set entire TCCR4B register to 0
  TCNT4  = 0; // initialize counter value to 0

  OCR4A = 312; //= (16MHz) / (50Hz*1024) - 1 (must be <65536)

  TCCR4B |= (1 << WGM12);
  TCCR4B |= (1 << CS12) | (1 << CS10); 
  TIMSK4 |= (1 << OCIE4A);
  sei();

  l_motor_controller.begin(&l_current_vel, &l_control_output, &l_target_vel, p_gain, i_gain, d_gain);  
  l_motor_controller.setOutputLimits(-255, 255);
  // l_motor_controller.setBias(255.0 / 2.0);
  l_motor_controller.setWindUpLimits(-5, 5);
  
  r_motor_controller.begin(&r_current_vel, &r_control_output, &r_target_vel, p_gain, i_gain, d_gain);  
  r_motor_controller.setOutputLimits(-255, 255);
  // r_motor_controller.setBias(255.0 / 2.0);
  r_motor_controller.setWindUpLimits(-5, 5);

  l_motor_controller.start();
  r_motor_controller.start();

  // Start communication
  Serial3.begin(115200); // DEBUG  
  Serial.begin(921600);
}

void loop() {
  
  uint8_t len = 0;
  len = process_protocol();

  if(len != 0)
  { 
    // response for command
    uint8_t cmd = g_recv_data[0];
    uint8_t need_res = g_recv_data[len - 1];

    if(cmd == 0x1)
    { 
      g_motor_enabled = g_recv_data[1];     

      if(need_res)
      {
        send_resonse_protocol(len);
      }
    }
    else if(cmd == 0x2)
    {
      g_l_motor_target = (int16_t)((g_recv_data[1] << 8) | g_recv_data[2]);
      g_r_motor_target = (int16_t)((g_recv_data[3] << 8) | g_recv_data[4]);

      g_l_lamp_val = g_recv_data[5];
      g_r_lamp_val = g_recv_data[6];      

      if(need_res)
      {
        send_resonse_protocol(len);
      }
    }
    else if(cmd == 0x03)
    {      
      send_current_state();     
    }

    g_rx_index = 0;      
  }    
  // delay(10);  
}

// Callback function for Timer4 Compare Interrupt
ISR(TIMER4_COMPA_vect)
{
  g_l_current_encoder = l_enc.read();
  g_r_current_encoder = r_enc.read();

  l_current_vel = g_l_current_encoder - g_l_last_encoder;
  r_current_vel = g_r_current_encoder - g_r_last_encoder;

  g_l_last_encoder = g_l_current_encoder;
  g_r_last_encoder = g_r_current_encoder;

  
  l_target_vel = g_l_motor_target / 50.0;
  r_target_vel = g_r_motor_target / 50.0;

  l_motor_controller.compute();
  r_motor_controller.compute();

  if(l_control_output >= 0)
  {
    analogWrite(9, l_control_output);
    analogWrite(10, 0);
  }
  else {
    analogWrite(9, 0);
    analogWrite(10, -1 * l_control_output);
  }

  if(r_control_output >= 0)
  {
    analogWrite(11, r_control_output);
    analogWrite(12, 0);
  }
  else {
    analogWrite(11, 0);
    analogWrite(12, -1 * r_control_output);
  }


  analogWrite(22, 255 - g_l_lamp_val);
  analogWrite(23, 255 - g_r_lamp_val);
  
  
  Serial3.print("EN: ");
  Serial3.print(g_motor_enabled);

  Serial3.print("  L_ENC:  ");
  Serial3.print(g_l_current_encoder);
  Serial3.print("  L_CUR:  ");
  Serial3.print(l_current_vel);
  Serial3.print("  L_TAR:  ");
  Serial3.print(l_target_vel);
  Serial3.print("  L_OUT:  ");
  Serial3.print(l_control_output);  

  Serial3.print("  RENC:  ");
  Serial3.print(g_r_current_encoder);
  Serial3.print("  R_CUR:  ");
  Serial3.print(r_current_vel);
  Serial3.print("  R_TAR:  ");
  Serial3.print(r_target_vel);
  Serial3.print("  R_OUT:  ");
  Serial3.print(r_control_output);
  Serial3.println("");
}


uint8_t process_protocol()
{
  if(Serial.available() > 0)
  {
    g_rx_buf[g_rx_index] = Serial.read();
    g_rx_index++;
  }

  if(g_rx_index > 6)
  {
    if(g_rx_buf[g_rx_index - 1] == 0xFD)
    {   
      if(g_rx_buf[g_rx_index - 2] = 0xFA)
      { 
        uint8_t packet_len = g_rx_buf[g_rx_index - 4];
        // Check Header
        if((g_rx_buf[g_rx_index - packet_len - 5] == 0xFE) && (g_rx_buf[g_rx_index -packet_len - 6] == 0xFA))
        { 
          // Check checksum
          uint8_t calc_crc = calc_checksum(&g_rx_buf[g_rx_index - packet_len - 4], packet_len + 1);       

          if(calc_crc == g_rx_buf[g_rx_index - 3])
          { 
            // Check completed.
            for(int i = 0; i < packet_len; i++)
            { 
              g_recv_data[i] = g_rx_buf[g_rx_index - packet_len - 4 + i];
            }

            memset(g_rx_buf, 0, 256);
            g_rx_index = 0;

            return packet_len;
          }
        }
      }      
    }
  }

  return 0;
}

void send_current_state(void)
{
  uint8_t send_data[16] = {0, };

  send_data[0] = 0xFA;
  send_data[1] = 0xFE;
  send_data[2] = 0x93;
  send_data[3] = g_motor_enabled;

  // int16_t l_state = int16_t(l_current_state * 50);  // enconder for second
  send_data[4] = (uint8_t)(g_l_current_encoder >> 24);
  send_data[5] = (uint8_t)(g_l_current_encoder >> 16);
  send_data[6] = (uint8_t)(g_l_current_encoder >> 8);
  send_data[7] = (uint8_t)(g_l_current_encoder);

  // int16_t r_state = int16_t(r_current_state * 50);  // enconder for second
  send_data[8] = (uint8_t)(g_r_current_encoder >> 24);
  send_data[9] = (uint8_t)(g_r_current_encoder >> 16);
  send_data[10] = (uint8_t)(g_r_current_encoder >> 8);
  send_data[11] = (uint8_t)(g_r_current_encoder);

  send_data[12] = g_l_lamp_val;
  send_data[13] = g_l_lamp_val;  

  send_data[14] = 12;

  int sum = 0;
  for(int i = 0; i < 12; i++)
  {
    sum += send_data[3+i];    
  }
  send_data[15] = (uint8_t)sum;

  send_data[16] = 0xFA;
  send_data[17] = 0xFD;

  Serial.write(send_data, 18);
}


void send_resonse_protocol(uint8_t len)
{
  uint8_t send_data[len+6];
  send_data[0] = 0xFA;
  send_data[1] = 0xFE;
  for(int i = 0; i < len; i++)
  {
    send_data[2+i] = g_recv_data[i];
  }
  send_data[2+len] = len;
  send_data[2] = send_data[2] + 0x90;
  
  uint16_t sum = 0;
  for(int i = 0; i < 2+len; i++)
  {
    sum += send_data[i];
  }
  send_data[3+len] = (uint8_t)sum;
  send_data[4+len] = 0xFA;
  send_data[5+len] = 0xFD;

  Serial.write(send_data, 6+len);
}


uint8_t calc_checksum(uint8_t* data, uint8_t len)
{
  uint16_t sum = 0;
  for(int i = 0; i < len; i++)
  {    
    sum += data[i];
  }

  return uint8_t(sum);
}
