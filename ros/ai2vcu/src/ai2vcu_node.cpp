#include <ros/ros.h>
#include <fs-ai_api.h>

char can_id[] = "can0";

int main(int argc, char** argv)
{
  int err;

  ros::init(argc, argv, "ai2vcu");
  err = fs_ai_api_init(can_id, 0, 0);
  if (err != EXIT_SUCCESS)
    return err;
  ros::spin();
}
