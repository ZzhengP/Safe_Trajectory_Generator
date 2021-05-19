// Generated by gencpp from file panda_mpc/UpdateTrajectoryNextPointResponse.msg
// DO NOT EDIT!


#ifndef PANDA_MPC_MESSAGE_UPDATETRAJECTORYNEXTPOINTRESPONSE_H
#define PANDA_MPC_MESSAGE_UPDATETRAJECTORYNEXTPOINTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace panda_mpc
{
template <class ContainerAllocator>
struct UpdateTrajectoryNextPointResponse_
{
  typedef UpdateTrajectoryNextPointResponse_<ContainerAllocator> Type;

  UpdateTrajectoryNextPointResponse_()
    : success(false)  {
    }
  UpdateTrajectoryNextPointResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> const> ConstPtr;

}; // struct UpdateTrajectoryNextPointResponse_

typedef ::panda_mpc::UpdateTrajectoryNextPointResponse_<std::allocator<void> > UpdateTrajectoryNextPointResponse;

typedef boost::shared_ptr< ::panda_mpc::UpdateTrajectoryNextPointResponse > UpdateTrajectoryNextPointResponsePtr;
typedef boost::shared_ptr< ::panda_mpc::UpdateTrajectoryNextPointResponse const> UpdateTrajectoryNextPointResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator1> & lhs, const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator1> & lhs, const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace panda_mpc

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "panda_mpc/UpdateTrajectoryNextPointResponse";
  }

  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UpdateTrajectoryNextPointResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::panda_mpc::UpdateTrajectoryNextPointResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PANDA_MPC_MESSAGE_UPDATETRAJECTORYNEXTPOINTRESPONSE_H
