#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <mavros_msgs/RotorControl.h>


namespace mavros {
namespace extra_plugins{

class RotorControlPlugin : public plugin::PluginBase {
public:
     RotorControlPlugin() : PluginBase(),
         nh("~rotorcontrol"){ };

     void initialize(UAS &uas_)
     {
         PluginBase::initialize(uas_);
         rotorcontrol_sub = nh.subscribe("rotor_control", 10, &RotorControlPlugin::rotorcontrol_cb, this);
     };

     Subscriptions get_subscriptions()
     {
         return {/* RX disabled */ };
     }

 private:
     ros::NodeHandle nh;
     ros::Subscriber rotorcontrol_sub;

    void rotorcontrol_cb(const mavros_msgs::RotorControl::ConstPtr &req)
     {
        mavros::UAS *m_uas_ = static_cast<RotorControlPlugin *>(this)->m_uas;

		mavlink::common::msg::ROTOR_CONTROL rotor_control = {}; 

		//官方教程中没有这句,所以会报错,因为send_message_ignore_drop的参数是message消息类型,
		//所以需要先用python mavgenerate.py生成mavlink::common::msg::KEY_COMMANDS
		rotor_control.control_mode = req->control_mode;
		rotor_control.thrust = req->thrust;
        for (int i=0; i<4;++i) {
           rotor_control.mot_throttle[i] = req->mot_throttle[i];
           rotor_control.rotor_thrust[i] = req->rotor_thrust[i];
        }
        // std::cout << "Got data1 : " << req->control_mode << req->thrust << std::endl;
        // std::cout << "Got data2 : " << req->mot_throttle[0] << req->mot_throttle[1] << req->mot_throttle[2] << req->mot_throttle[3]<< std::endl;
        // std::cout << "Got data3 : " << req->rotor_thrust[0] << req->rotor_thrust[1] << req->rotor_thrust[2] << req->rotor_thrust[3]<< std::endl;
        UAS_FCU(m_uas)->send_message_ignore_drop(rotor_control);

     }
};
}   // namespace extra_plugins
}   // namespace mavros
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RotorControlPlugin, mavros::plugin::PluginBase)