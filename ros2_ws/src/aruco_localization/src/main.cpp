
#include <memory>                                                                                                                      
                                                                                                                                       
#include "rclcpp/rclcpp.hpp"                                                                                                           
#include "aruco_localization/aruco_localization_node.hpp"
                                                                                                                                       
int main(int argc, char ** argv)                                                                                                       
{                                                                                                                                      
  rclcpp::init(argc, argv);                                                                                                            
  rclcpp::spin(std::make_shared<ArucoLocalization>());                                                                                       
  rclcpp::shutdown();                                                                                                                  
                                                                                                                                       
  return 0;                                                                                                                            
} 