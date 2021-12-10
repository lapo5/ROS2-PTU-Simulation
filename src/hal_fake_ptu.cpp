#include <iostream>
#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ptu_interfaces/msg/ptu.hpp"
#include "ptu_interfaces/srv/set_pan.hpp"
#include "ptu_interfaces/srv/set_tilt.hpp"
#include "ptu_interfaces/srv/set_pan_tilt.hpp"
#include "ptu_interfaces/srv/set_pan_tilt_speed.hpp"
#include "ptu_interfaces/srv/get_limits.hpp"

#include "ptu_interfaces/action/set_pan.hpp"
#include "ptu_interfaces/action/set_tilt.hpp"
#include "ptu_interfaces/action/set_pan_tilt.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
namespace ph = std::placeholders;

class HALFakePTU : public rclcpp::Node {
 public:
    using SetPanAction = ptu_interfaces::action::SetPan;
    using GoalHandlePanAction = rclcpp_action::ServerGoalHandle<SetPanAction>;

    using SetTiltAction = ptu_interfaces::action::SetTilt;
    using GoalHandleTiltAction = rclcpp_action::ServerGoalHandle<SetTiltAction>;

    using SetPanTiltAction = ptu_interfaces::action::SetPanTilt;
    using GoalHandlePanTiltAction = rclcpp_action::ServerGoalHandle<SetPanTiltAction>;

    HALFakePTU() : Node("hal_fake_ptu") {}
    
    bool init() {

        current_pan = 0.0;
        current_tilt = 0.0;

        tilt_min = declare_parameter("limits.min_tilt", -0.5);
        tilt_max = declare_parameter("limits.max_tilt", 0.5);
        tilt_speed = declare_parameter("limits.tilt_speed", 0.1);
        
        pan_min = declare_parameter("limits.min_pan", -1.0);
        pan_max = declare_parameter("limits.max_pan", 1.0);
        pan_speed = declare_parameter("limits.pan_speed", 0.1);

        min_step = declare_parameter("ptu_resolution", 0.1);
        internal_rate = declare_parameter("internal_rate", 100.0);

        min_threshold_to_move_pan = declare_parameter("min_thresold_command_input_pan", 0.001);
        min_threshold_to_move_tilt = declare_parameter("min_thresold_command_input_tilt", 0.001);

        std::string ptu_state_publisher = declare_parameter<std::string>("publishers.state", "/ptu/state");
        std::string set_pan_srv_name = declare_parameter<std::string>("services.set_pan", "/ptu/set_pan");
        std::string set_tilt_srv_name = declare_parameter<std::string>("services.set_tilt", "/ptu/set_tilt");
        std::string set_pantilt_srv_name = declare_parameter<std::string>("services.set_pantilt", "/ptu/set_pan_tilt");
        std::string set_pantilt_speed_srv_name = declare_parameter<std::string>("services.set_pantilt_speed", "/ptu/set_pan_tilt_speed");
        std::string set_reset_srv_name = declare_parameter<std::string>("services.reset", "/ptu/reset");
        std::string set_get_limits_srv_name = declare_parameter<std::string>("services.get_limits", "/ptu/get_limits");

        std::string set_pan_action_name = declare_parameter<std::string>("actions.set_pan", "/ptu/set_pan");
        std::string set_tilt_action_name = declare_parameter<std::string>("actions.set_tilt", "/ptu/set_tilt");
        std::string set_pantilt_action_name = declare_parameter<std::string>("actions.set_pantilt", "/ptu/set_pan_tilt");

        ptu_state_pub = create_publisher<ptu_interfaces::msg::PTU>(ptu_state_publisher, 1);

        set_pan_srv = create_service<ptu_interfaces::srv::SetPan>(set_pan_srv_name, std::bind(&HALFakePTU::set_pan_callback, this, std::placeholders::_1, std::placeholders::_2));    

        set_tilt_srv = create_service<ptu_interfaces::srv::SetTilt>(set_tilt_srv_name, std::bind(&HALFakePTU::set_tilt_callback, this, std::placeholders::_1, std::placeholders::_2));    

        set_pantilt_srv = create_service<ptu_interfaces::srv::SetPanTilt>(set_pantilt_srv_name, std::bind(&HALFakePTU::set_pantilt_callback, this, std::placeholders::_1, std::placeholders::_2));    

        set_pantilt_speed_srv = create_service<ptu_interfaces::srv::SetPanTiltSpeed>(set_pantilt_speed_srv_name, std::bind(&HALFakePTU::set_pantilt_speed_callback, this, std::placeholders::_1, std::placeholders::_2));    

        reset_srv = create_service<std_srvs::srv::Empty>(set_reset_srv_name, std::bind(&HALFakePTU::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

        get_limits_srv = create_service<ptu_interfaces::srv::GetLimits>(set_get_limits_srv_name, std::bind(&HALFakePTU::get_limits_callback, this, std::placeholders::_1, std::placeholders::_2));

        this->action_server_set_pan = rclcpp_action::create_server<SetPanAction>(
            this,
            set_pan_action_name,
            std::bind(&HALFakePTU::handle_goal_pan, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&HALFakePTU::handle_cancel_pan, this, std::placeholders::_1),
            std::bind(&HALFakePTU::handle_accepted_pan, this, std::placeholders::_1)
        );


        this->action_server_set_tilt = rclcpp_action::create_server<SetTiltAction>(
            this,
            set_tilt_action_name,
            std::bind(&HALFakePTU::handle_goal_tilt, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&HALFakePTU::handle_cancel_tilt, this, std::placeholders::_1),
            std::bind(&HALFakePTU::handle_accepted_tilt, this, std::placeholders::_1)
        );


        this->action_server_set_pantilt = rclcpp_action::create_server<SetPanTiltAction>(
            this,
            set_pantilt_action_name,
            std::bind(&HALFakePTU::handle_goal_pantilt, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&HALFakePTU::handle_cancel_pantilt, this, std::placeholders::_1),
            std::bind(&HALFakePTU::handle_accepted_pantilt, this, std::placeholders::_1)
        );

        hz = declare_parameter("hz", 10.0);

        timer_ = this->create_wall_timer(1000ms / hz, std::bind(&HALFakePTU::spinCallback, this));
      
        RCLCPP_INFO_STREAM(get_logger(), "[FAKE PTU] Node Ready");
        return true;
    }

    ~HALFakePTU(){
        
    }


 private:
    double current_pan, current_tilt;
    double default_velocity_;

    double pan_min, pan_max, tilt_min, tilt_max;

    double pan_speed, tilt_speed;
    double hz;

    double min_step;
    double internal_rate;

    double min_threshold_to_move_pan, min_threshold_to_move_tilt;
    
    rclcpp::Publisher<ptu_interfaces::msg::PTU>::SharedPtr ptu_state_pub;

    rclcpp::Service<ptu_interfaces::srv::SetPan>::SharedPtr set_pan_srv;
    rclcpp::Service<ptu_interfaces::srv::SetTilt>::SharedPtr set_tilt_srv;
    rclcpp::Service<ptu_interfaces::srv::SetPanTilt>::SharedPtr set_pantilt_srv;

    rclcpp_action::Server<SetPanAction>::SharedPtr action_server_set_pan;
    rclcpp_action::Server<SetTiltAction>::SharedPtr action_server_set_tilt;
    rclcpp_action::Server<SetPanTiltAction>::SharedPtr action_server_set_pantilt;

    rclcpp::Service<ptu_interfaces::srv::SetPanTiltSpeed>::SharedPtr set_pantilt_speed_srv;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
    rclcpp::Service<ptu_interfaces::srv::GetLimits>::SharedPtr get_limits_srv;

    rclcpp::TimerBase::SharedPtr timer_;


    void get_limits_callback(const std::shared_ptr<ptu_interfaces::srv::GetLimits::Request>,
            std::shared_ptr<ptu_interfaces::srv::GetLimits::Response> response){
        response->pan_min= this->pan_min;
        response->tilt_min = this->tilt_min;
        response->pan_max = this->pan_max;
        response->tilt_max = this->tilt_max;
    }


    void resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
            std::shared_ptr<std_srvs::srv::Empty::Response>){
        current_pan = 0.0;
        current_tilt = 0.0;
    }


    void set_pan_callback(const std::shared_ptr<ptu_interfaces::srv::SetPan::Request> request,
            std::shared_ptr<ptu_interfaces::srv::SetPan::Response>      response){

        double loop_time_sec = 1.0/internal_rate;
        rclcpp::Rate loop_rate(internal_rate);
        RCLCPP_INFO_STREAM(get_logger(), "[FAKE PTU] set_pan_callback");
                
        if(abs(request->pan - current_pan) > min_threshold_to_move_pan)
        {
            double step;
            while(1){
                step = pan_speed * (request->pan - current_pan);
                if (abs(step) < this->min_step)
                {
                    if (step > 0)
                        step = this->min_step;
                    else
                        step = -1 * this->min_step;
                }

                current_pan = current_pan + loop_time_sec * step;

                RCLCPP_INFO_STREAM(get_logger(), "[FAKE PTU] current_pan:  " << current_pan);
                if(abs(request->pan - current_pan) < min_threshold_to_move_pan)
                {
                    break;
                }
               
                loop_rate.sleep();

            }
        }
        response->ret = true;
    }


    void set_tilt_callback(const std::shared_ptr<ptu_interfaces::srv::SetTilt::Request> request,
            std::shared_ptr<ptu_interfaces::srv::SetTilt::Response>      response){

        double loop_time_sec = 1.0/internal_rate;
        rclcpp::Rate loop_rate(internal_rate);
        
        if(abs(request->tilt - current_tilt) > min_threshold_to_move_tilt)
        {
            double step;

            while(1){
                step = tilt_speed * (request->tilt - current_tilt);
                if (abs(step) < this->min_step)
                {
                    if (step > 0)
                        step = this->min_step;
                    else
                        step = -1 * this->min_step;
                }

                current_tilt = current_tilt + loop_time_sec * step;

                RCLCPP_INFO_STREAM(get_logger(), "[FAKE PTU] current_tilt:  " << current_tilt);
                if(abs(request->tilt - current_tilt) < min_threshold_to_move_pan)
                {
                    break;
                }
               
                loop_rate.sleep();

            }
        }
        
        response->ret = true;
    }


    void set_pantilt_callback(const std::shared_ptr<ptu_interfaces::srv::SetPanTilt::Request> request,
            std::shared_ptr<ptu_interfaces::srv::SetPanTilt::Response>      response){

        double loop_time_sec = 1.0/internal_rate;
        rclcpp::Rate loop_rate(internal_rate);

        if(abs(request->pan - current_pan) > min_threshold_to_move_pan)
        {
            double step;
            while(1){
                step = pan_speed * (request->pan - current_pan);
                if (abs(step) < this->min_step)
                {
                    if (step > 0)
                        step = this->min_step;
                    else
                        step = -1 * this->min_step;
                }

                current_pan = current_pan + loop_time_sec * step;
                RCLCPP_INFO_STREAM(get_logger(), "[FAKE PTU] current_pan:  " << current_pan);

                if(abs(request->pan - current_pan) < min_threshold_to_move_pan)
                {
                    break;
                }
               
                loop_rate.sleep();

            }
        }

        if(abs(request->tilt - current_tilt) > min_threshold_to_move_tilt)
        {
            double step;
            while(1){

                step = tilt_speed * (request->tilt - current_tilt);
                if (abs(step) < this->min_step)
                {
                    if (step > 0)
                        step = this->min_step;
                    else
                        step = -1 * this->min_step;
                }

                current_tilt = current_tilt + loop_time_sec * step;
                RCLCPP_INFO_STREAM(get_logger(), "[FAKE PTU] current_tilt:  " << current_tilt);

                if(abs(request->tilt - current_tilt) < min_threshold_to_move_tilt)
                {
                    break;
                }
               
                loop_rate.sleep();

            }
        }
        
        response->ret = true;
    }


    void set_pantilt_speed_callback(const std::shared_ptr<ptu_interfaces::srv::SetPanTiltSpeed::Request> request,
            std::shared_ptr<ptu_interfaces::srv::SetPanTiltSpeed::Response>      response){

        pan_speed = request->pan_speed;
        tilt_speed = request->tilt_speed;

        response->ret = true;
    }

    
    void spinCallback(){
        // Publish Position & Speed
        ptu_interfaces::msg::PTU ptu_msg;
        ptu_msg.header.stamp = now();
        ptu_msg.pan = current_pan;
        ptu_msg.tilt = current_tilt;
        ptu_msg.pan_speed = pan_speed;
        ptu_msg.tilt_speed = tilt_speed;
        ptu_state_pub->publish(ptu_msg);
    }


    rclcpp_action::GoalResponse handle_goal_pan(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SetPanAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handle_cancel_pan(
        const std::shared_ptr<GoalHandlePanAction> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handle_accepted_pan(const std::shared_ptr<GoalHandlePanAction> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&HALFakePTU::execute_pan_action, this, _1), goal_handle}.detach();
    }


    void execute_pan_action(const std::shared_ptr<GoalHandlePanAction> goal_handle)
    {
        double loop_time_sec = 1.0/internal_rate;
        rclcpp::Rate loop_rate(internal_rate);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SetPanAction::Feedback>();
        double perc_of_compl = 0.0;

        auto result = std::make_shared<SetPanAction::Result>();

        double excursion = abs(goal->pan - current_pan);
        if(excursion > min_threshold_to_move_pan)
        {
            double step;
            while(1){

                step = pan_speed * (goal->pan - current_pan);
                if (abs(step) < this->min_step)
                {
                    if (step > 0)
                        step = this->min_step;
                    else
                        step = -1 * this->min_step;
                }

                current_pan = current_pan + loop_time_sec * step;

                if (goal_handle->is_canceling()) {
                    result->ret = false;
                    goal_handle->canceled(result);
                    return;
                }

                if(abs(goal->pan - current_pan) < min_threshold_to_move_pan)
                {
                    break;
                }
                perc_of_compl = 100.0 - ((abs(goal->pan - current_pan) / excursion * 100.0));
                feedback->percentage_of_completing = perc_of_compl;
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();

            }

        }

        if (rclcpp::ok()) {
            result->ret = true;
            goal_handle->succeed(result);
        }
    }


    rclcpp_action::GoalResponse handle_goal_tilt(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SetTiltAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handle_cancel_tilt(
        const std::shared_ptr<GoalHandleTiltAction> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handle_accepted_tilt(const std::shared_ptr<GoalHandleTiltAction> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&HALFakePTU::execute_tilt_action, this, _1), goal_handle}.detach();
    }


    void execute_tilt_action(const std::shared_ptr<GoalHandleTiltAction> goal_handle)
    {
        double loop_time_sec = 1.0/internal_rate;
        rclcpp::Rate loop_rate(internal_rate);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SetTiltAction::Feedback>();
        double perc_of_compl = 0.0;

        auto result = std::make_shared<SetTiltAction::Result>();

        double excursion = abs(goal->tilt - current_tilt);
        if(excursion > min_threshold_to_move_tilt)
        {
            double step;
            while(1){

                step = tilt_speed * (goal->tilt - current_tilt);
                if (abs(step) < this->min_step)
                {
                    if (step > 0)
                        step = this->min_step;
                    else
                        step = -1 * this->min_step;
                }

                current_tilt = current_tilt + loop_time_sec * step;

                if (goal_handle->is_canceling()) {
                    result->ret = false;
                    goal_handle->canceled(result);
                    return;
                }

                if(abs(goal->tilt - current_tilt) < min_threshold_to_move_tilt)
                {
                    break;
                }

                perc_of_compl = 100.0 - (abs(goal->tilt - current_tilt) / excursion * 100.0);
                feedback->percentage_of_completing = perc_of_compl;
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();

            }

        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->ret = true;
            goal_handle->succeed(result);
        }
    }


    rclcpp_action::GoalResponse handle_goal_pantilt(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SetPanTiltAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handle_cancel_pantilt(
        const std::shared_ptr<GoalHandlePanTiltAction> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handle_accepted_pantilt(const std::shared_ptr<GoalHandlePanTiltAction> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&HALFakePTU::execute_pantilt_action, this, _1), goal_handle}.detach();
    }
    

    void execute_pantilt_action(const std::shared_ptr<GoalHandlePanTiltAction> goal_handle)
    {
        double loop_time_sec = 1.0/internal_rate;
        rclcpp::Rate loop_rate(internal_rate);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SetPanTiltAction::Feedback>();
        double perc_of_compl_pan = 0.0;
        double perc_of_compl_tilt = 0.0;

        auto result = std::make_shared<SetPanTiltAction::Result>();

        double excursion_pan = abs(goal->pan - current_pan);
        double excursion_tilt = abs(goal->tilt - current_tilt);
        
        if(excursion_pan > min_threshold_to_move_pan or excursion_tilt > min_threshold_to_move_tilt)
        {
            double step_pan, step_tilt;
            while(1){
                step_pan = loop_time_sec * (goal->pan - current_pan);
                if (abs(step_pan) < this->min_step)
                {
                    if (step_pan > 0)
                        step_pan = this->min_step;
                    else
                        step_pan = -1 * this->min_step;
                }

                step_tilt = loop_time_sec * (goal->tilt - current_tilt);
                if (abs(step_tilt) < this->min_step)
                {
                    if (step_tilt > 0)
                        step_tilt = this->min_step;
                    else
                        step_tilt = -1 * this->min_step;
                }


                if (goal_handle->is_canceling()) {
                    result->ret = false;
                    goal_handle->canceled(result);
                    return;
                }

                if(abs(goal->tilt - current_tilt) >= min_threshold_to_move_tilt)
                    current_tilt = current_tilt + tilt_speed * step_tilt;
                

                if(abs(goal->pan - current_pan) >= min_threshold_to_move_pan)
                    current_pan = current_pan + pan_speed * step_pan;

                if(abs(goal->pan - current_pan) < min_threshold_to_move_pan and abs(goal->tilt - current_tilt) < min_threshold_to_move_tilt)
                {
                    break;
                }

                if (abs(goal->pan - current_pan) < min_threshold_to_move_pan)
                    perc_of_compl_pan = 100.0;
                else
                    perc_of_compl_pan = 100.0 - (abs(goal->pan - current_pan) / excursion_pan * 100.0);
                
                if (abs(goal->tilt - current_tilt) < min_threshold_to_move_tilt)
                    perc_of_compl_tilt = 100.0;
                else
                    perc_of_compl_tilt = 100.0 - (abs(goal->tilt - current_tilt) / excursion_tilt * 100.0);
                
                feedback->percentage_of_completing_pan = perc_of_compl_pan;
                feedback->percentage_of_completing_tilt = perc_of_compl_tilt;

                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();

            }

        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->ret = true;
            goal_handle->succeed(result);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<HALFakePTU>();
    if (not node->init()) rclcpp::shutdown();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
