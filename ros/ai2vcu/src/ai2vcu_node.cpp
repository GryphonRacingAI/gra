#include <ros/ros.h>
#include <fs-ai_api.h>
#include <ackermann_msgs/AckermannDrive.h>

#include <ai2vcu/VCU2AI.h>

char can_id[] = "can0";

class FSAIAPI {
private:
  fs_ai_api_ai2vcu m_tx_data;
  fs_ai_api_vcu2ai m_rx_data;

  static fs_ai_api_ai2vcu_struct tx_data_init() {
    fs_ai_api_ai2vcu_struct ret;
    ret.AI2VCU_MISSION_STATUS = MISSION_NOT_SELECTED;
    ret.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
    ret.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
    ret.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
    ret.AI2VCU_STEER_ANGLE_REQUEST_deg = 0.0;
    ret.AI2VCU_AXLE_SPEED_REQUEST_rpm = 0.0;
    ret.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0.0;
    ret.AI2VCU_BRAKE_PRESS_REQUEST_pct = 0.0;
    return ret;
  }

public:
  FSAIAPI(char *can_id, bool debug=false, bool simulate=false)
      : m_tx_data(tx_data_init()) {
    int err;
    err = fs_ai_api_init(can_id, debug, simulate);
    if (err != EXIT_SUCCESS)
      throw err;

    transmit();
    receive();
  }

  // Ideally, this function should be called every 10ms.
  void transmit() {
    fs_ai_api_ai2vcu_set_data(&m_tx_data);
  }

  void receive() {
    fs_ai_api_vcu2ai_get_data(&m_rx_data);
  }

  void set_mission_status(fs_ai_api_mission_status_e mission_status) {
    m_tx_data.AI2VCU_MISSION_STATUS = mission_status;
  }

  void set_direction_request(fs_ai_api_direction_request_e direction_request) {
    m_tx_data.AI2VCU_DIRECTION_REQUEST = direction_request;
  }

  void set_estop_request(fs_ai_api_estop_request_e estop_request) {
    m_tx_data.AI2VCU_ESTOP_REQUEST = estop_request;
  }

  void set_handshake_send_bit(fs_ai_api_handshake_send_bit_e handshake_send_bit) {
    m_tx_data.AI2VCU_HANDSHAKE_SEND_BIT = handshake_send_bit;
  }

  void set_steer_angle_request(float steer_angle_request) {
    m_tx_data.AI2VCU_STEER_ANGLE_REQUEST_deg = steer_angle_request;
  }

  void set_axle_speed_request(float axle_speed_request) {
    m_tx_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = axle_speed_request;
  }

  void set_axle_torque_request(float axle_torque_request) {
    m_tx_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = axle_torque_request;
  }

  void set_brake_press_request(float brake_press_request) {
    m_tx_data.AI2VCU_BRAKE_PRESS_REQUEST_pct = brake_press_request;
  }

  fs_ai_api_handshake_receive_bit_e get_handshake_receive_bit() {
    return m_rx_data.VCU2AI_HANDSHAKE_RECEIVE_BIT;
  }

  fs_ai_api_res_go_signal_bit_e get_res_go_signal_bit() {
    return m_rx_data.VCU2AI_RES_GO_SIGNAL;
  }

  fs_ai_api_as_state_e get_as_state() {
    return m_rx_data.VCU2AI_AS_STATE;
  }

  fs_ai_api_ami_state_e get_ami_state() {
    return m_rx_data.VCU2AI_AMI_STATE;
  }

  float get_steer_angle() {
    return m_rx_data.VCU2AI_STEER_ANGLE_deg;
  }

  float get_brake_press_front() {
    return m_rx_data.VCU2AI_BRAKE_PRESS_F_pct;
  }

  float get_brake_press_rear() {
    return m_rx_data.VCU2AI_BRAKE_PRESS_R_pct;
  }

  float get_wheel_speed_front_left() {
    return m_rx_data.VCU2AI_FL_WHEEL_SPEED_rpm;
  }

  float get_wheel_speed_front_right() {
    return m_rx_data.VCU2AI_FR_WHEEL_SPEED_rpm;
  }

  float get_wheel_speed_rear_left() {
    return m_rx_data.VCU2AI_RL_WHEEL_SPEED_rpm;
  }

  float get_wheel_speed_rear_right() {
    return m_rx_data.VCU2AI_RR_WHEEL_SPEED_rpm;
  }

  uint16_t get_pulse_count_front_left() {
    return m_rx_data.VCU2AI_FL_PULSE_COUNT;
  }

  uint16_t get_pulse_count_front_right() {
    return m_rx_data.VCU2AI_FR_PULSE_COUNT;
  }

  uint16_t get_pulse_count_rear_left() {
    return m_rx_data.VCU2AI_RL_PULSE_COUNT;
  }

  uint16_t get_pulse_count_rear_right() {
    return m_rx_data.VCU2AI_RR_PULSE_COUNT;
  }
};

class AI2VCUNode {
private:
  FSAIAPI m_fs_ai_api;
  ros::NodeHandle m_node_handle;
  ros::Rate m_rate;
  ros::Publisher m_publisher;
  ros::Subscriber m_subscriber_ackermann;

  ai2vcu::VCU2AI get_message() {
    ai2vcu::VCU2AI msg;
    msg.handshake_receive_bit = m_fs_ai_api.get_handshake_receive_bit();
    msg.res_go_signal = m_fs_ai_api.get_res_go_signal_bit();
    msg.as_state = m_fs_ai_api.get_as_state();
    msg.ami_state = m_fs_ai_api.get_ami_state();
    msg.steer_angle_deg = m_fs_ai_api.get_steer_angle();
    msg.brake_press_f_pct = m_fs_ai_api.get_brake_press_front();
    msg.brake_press_r_pct = m_fs_ai_api.get_brake_press_rear();
    msg.fl_wheel_speed_rpm = m_fs_ai_api.get_wheel_speed_front_left();
    msg.fr_wheel_speed_rpm = m_fs_ai_api.get_wheel_speed_front_right();
    msg.rl_wheel_speed_rpm = m_fs_ai_api.get_wheel_speed_rear_left();
    msg.rr_wheel_speed_rpm = m_fs_ai_api.get_wheel_speed_rear_right();
    msg.fl_pulse_count = m_fs_ai_api.get_pulse_count_front_left();
    msg.fr_pulse_count = m_fs_ai_api.get_pulse_count_front_right();
    msg.rl_pulse_count = m_fs_ai_api.get_pulse_count_rear_left();
    msg.rr_pulse_count = m_fs_ai_api.get_pulse_count_rear_right();
    return msg;
  }

  void ackermann_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg) {
    m_fs_ai_api.set_direction_request(msg->speed > 0 ? DIRECTION_FORWARD : DIRECTION_NEUTRAL);
    m_fs_ai_api.set_steer_angle_request(msg->steering_angle);
    m_fs_ai_api.set_axle_speed_request(msg->speed);
    m_fs_ai_api.set_axle_torque_request(msg->speed > 0 ? 250 : 0);
  }

public:
  AI2VCUNode(char *can_id): m_fs_ai_api(can_id), m_rate(100) {
    m_publisher = m_node_handle.advertise<ai2vcu::VCU2AI>("vcu2ai", 100);
    m_subscriber_ackermann = m_node_handle.subscribe("ackermann_cmd", 1000, &AI2VCUNode::ackermann_callback, this);
  }

  void spin() {
    while (ros::ok()) {
      m_fs_ai_api.transmit();
      m_fs_ai_api.receive();

      m_publisher.publish(get_message());

      m_rate.sleep();
      ros::spinOnce();
    }
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ai2vcu");
  AI2VCUNode node(can_id);
  node.spin();
}
