#include "ros/ros.h"
#include "line_tracer/white_pixel.h"

// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void chatterCallback(const line_tracer::white_pixel msg)
{
    printf("white_count: %d\n",msg.white_count);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "p_control");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("white_count", 1000, chatterCallback);

  // トピック更新の待ちうけを行うAPI
  ros::spin();

  return 0;
}
