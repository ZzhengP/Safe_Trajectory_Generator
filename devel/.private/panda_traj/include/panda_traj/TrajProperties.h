// Generated by gencpp from file panda_traj/TrajProperties.msg
// DO NOT EDIT!


#ifndef PANDA_TRAJ_MESSAGE_TRAJPROPERTIES_H
#define PANDA_TRAJ_MESSAGE_TRAJPROPERTIES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose.h>

namespace panda_traj
{
template <class ContainerAllocator>
struct TrajProperties_
{
  typedef TrajProperties_<ContainerAllocator> Type;

  TrajProperties_()
    : play_traj_(false)
    , jogging_(false)
    , gain_tunning_(false)
    , move_(false)
    , index_(0)
    , amplitude(0.0)
    , X_curr_()
    , X_des_jog_()  {
    }
  TrajProperties_(const ContainerAllocator& _alloc)
    : play_traj_(false)
    , jogging_(false)
    , gain_tunning_(false)
    , move_(false)
    , index_(0)
    , amplitude(0.0)
    , X_curr_(_alloc)
    , X_des_jog_(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _play_traj__type;
  _play_traj__type play_traj_;

   typedef uint8_t _jogging__type;
  _jogging__type jogging_;

   typedef uint8_t _gain_tunning__type;
  _gain_tunning__type gain_tunning_;

   typedef uint8_t _move__type;
  _move__type move_;

   typedef int64_t _index__type;
  _index__type index_;

   typedef double _amplitude_type;
  _amplitude_type amplitude;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _X_curr__type;
  _X_curr__type X_curr_;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _X_des_jog__type;
  _X_des_jog__type X_des_jog_;





  typedef boost::shared_ptr< ::panda_traj::TrajProperties_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::panda_traj::TrajProperties_<ContainerAllocator> const> ConstPtr;

}; // struct TrajProperties_

typedef ::panda_traj::TrajProperties_<std::allocator<void> > TrajProperties;

typedef boost::shared_ptr< ::panda_traj::TrajProperties > TrajPropertiesPtr;
typedef boost::shared_ptr< ::panda_traj::TrajProperties const> TrajPropertiesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::panda_traj::TrajProperties_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::panda_traj::TrajProperties_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::panda_traj::TrajProperties_<ContainerAllocator1> & lhs, const ::panda_traj::TrajProperties_<ContainerAllocator2> & rhs)
{
  return lhs.play_traj_ == rhs.play_traj_ &&
    lhs.jogging_ == rhs.jogging_ &&
    lhs.gain_tunning_ == rhs.gain_tunning_ &&
    lhs.move_ == rhs.move_ &&
    lhs.index_ == rhs.index_ &&
    lhs.amplitude == rhs.amplitude &&
    lhs.X_curr_ == rhs.X_curr_ &&
    lhs.X_des_jog_ == rhs.X_des_jog_;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::panda_traj::TrajProperties_<ContainerAllocator1> & lhs, const ::panda_traj::TrajProperties_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace panda_traj

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::panda_traj::TrajProperties_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::panda_traj::TrajProperties_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::panda_traj::TrajProperties_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::panda_traj::TrajProperties_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::panda_traj::TrajProperties_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::panda_traj::TrajProperties_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::panda_traj::TrajProperties_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8fb34236d88ea1e31629703f4e635b92";
  }

  static const char* value(const ::panda_traj::TrajProperties_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8fb34236d88ea1e3ULL;
  static const uint64_t static_value2 = 0x1629703f4e635b92ULL;
};

template<class ContainerAllocator>
struct DataType< ::panda_traj::TrajProperties_<ContainerAllocator> >
{
  static const char* value()
  {
    return "panda_traj/TrajProperties";
  }

  static const char* value(const ::panda_traj::TrajProperties_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::panda_traj::TrajProperties_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool play_traj_\n"
"bool jogging_\n"
"bool gain_tunning_\n"
"bool move_\n"
"int64 index_\n"
"float64 amplitude\n"
"geometry_msgs/Pose X_curr_\n"
"geometry_msgs/Pose X_des_jog_\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::panda_traj::TrajProperties_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::panda_traj::TrajProperties_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.play_traj_);
      stream.next(m.jogging_);
      stream.next(m.gain_tunning_);
      stream.next(m.move_);
      stream.next(m.index_);
      stream.next(m.amplitude);
      stream.next(m.X_curr_);
      stream.next(m.X_des_jog_);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrajProperties_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::panda_traj::TrajProperties_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::panda_traj::TrajProperties_<ContainerAllocator>& v)
  {
    s << indent << "play_traj_: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.play_traj_);
    s << indent << "jogging_: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.jogging_);
    s << indent << "gain_tunning_: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gain_tunning_);
    s << indent << "move_: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.move_);
    s << indent << "index_: ";
    Printer<int64_t>::stream(s, indent + "  ", v.index_);
    s << indent << "amplitude: ";
    Printer<double>::stream(s, indent + "  ", v.amplitude);
    s << indent << "X_curr_: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.X_curr_);
    s << indent << "X_des_jog_: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.X_des_jog_);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PANDA_TRAJ_MESSAGE_TRAJPROPERTIES_H