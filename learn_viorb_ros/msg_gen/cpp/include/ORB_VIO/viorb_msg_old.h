/* Auto-generated by genmsg_cpp for file /home/sicong/VIORB/ORB_SLAM2/Examples/ROS/ORB_VIO/msg/viorb_msg.msg */
#ifndef ORB_VIO_MESSAGE_VIORB_MSG_H
#define ORB_VIO_MESSAGE_VIORB_MSG_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

namespace ORB_VIO
{
template <class ContainerAllocator>
struct viorb_msg_ {
  typedef viorb_msg_<ContainerAllocator> Type;

  viorb_msg_()
  : header()
  , Tic()
  , Qwi()
  , gw()
  , VINSInitFlag()
  , TrackStatus()
  {
  }

  viorb_msg_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , Tic(_alloc)
  , Qwi(_alloc)
  , gw(_alloc)
  , VINSInitFlag(_alloc)
  , TrackStatus(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _Tic_type;
   ::geometry_msgs::Pose_<ContainerAllocator>  Tic;

  typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _Qwi_type;
   ::geometry_msgs::Quaternion_<ContainerAllocator>  Qwi;

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _gw_type;
   ::geometry_msgs::Point_<ContainerAllocator>  gw;

  typedef  ::std_msgs::Bool_<ContainerAllocator>  _VINSInitFlag_type;
   ::std_msgs::Bool_<ContainerAllocator>  VINSInitFlag;

  typedef  ::std_msgs::Int8_<ContainerAllocator>  _TrackStatus_type;
   ::std_msgs::Int8_<ContainerAllocator>  TrackStatus;


  typedef boost::shared_ptr< ::ORB_VIO::viorb_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ORB_VIO::viorb_msg_<ContainerAllocator>  const> ConstPtr;
}; // struct viorb_msg
typedef  ::ORB_VIO::viorb_msg_<std::allocator<void> > viorb_msg;

typedef boost::shared_ptr< ::ORB_VIO::viorb_msg> viorb_msgPtr;
typedef boost::shared_ptr< ::ORB_VIO::viorb_msg const> viorb_msgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ORB_VIO::viorb_msg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ORB_VIO::viorb_msg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ORB_VIO

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ORB_VIO::viorb_msg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ORB_VIO::viorb_msg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ORB_VIO::viorb_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "22701a4895958bd1b68178837a5aa9f2";
  }

  static const char* value(const  ::ORB_VIO::viorb_msg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x22701a4895958bd1ULL;
  static const uint64_t static_value2 = 0xb68178837a5aa9f2ULL;
};

template<class ContainerAllocator>
struct DataType< ::ORB_VIO::viorb_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ORB_VIO/viorb_msg";
  }

  static const char* value(const  ::ORB_VIO::viorb_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ORB_VIO::viorb_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
geometry_msgs/Pose Tic\n\
geometry_msgs/Quaternion Qwi\n\
geometry_msgs/Point gw\n\
std_msgs/Bool VINSInitFlag\n\
std_msgs/Int8 TrackStatus\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: std_msgs/Bool\n\
bool data\n\
================================================================================\n\
MSG: std_msgs/Int8\n\
int8 data\n\
\n\
";
  }

  static const char* value(const  ::ORB_VIO::viorb_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::ORB_VIO::viorb_msg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::ORB_VIO::viorb_msg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ORB_VIO::viorb_msg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.Tic);
    stream.next(m.Qwi);
    stream.next(m.gw);
    stream.next(m.VINSInitFlag);
    stream.next(m.TrackStatus);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct viorb_msg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ORB_VIO::viorb_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ORB_VIO::viorb_msg_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "Tic: ";
s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.Tic);
    s << indent << "Qwi: ";
s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.Qwi);
    s << indent << "gw: ";
s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.gw);
    s << indent << "VINSInitFlag: ";
s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.VINSInitFlag);
    s << indent << "TrackStatus: ";
s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.TrackStatus);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ORB_VIO_MESSAGE_VIORB_MSG_H

