// Generated by gencpp from file panda_mpc/UpdateTrajectoryNextPoint.msg
// DO NOT EDIT!


#ifndef PANDA_MPC_MESSAGE_UPDATETRAJECTORYNEXTPOINT_H
#define PANDA_MPC_MESSAGE_UPDATETRAJECTORYNEXTPOINT_H

#include <ros/service_traits.h>


#include <panda_mpc/UpdateTrajectoryNextPointRequest.h>
#include <panda_mpc/UpdateTrajectoryNextPointResponse.h>


namespace panda_mpc
{

struct UpdateTrajectoryNextPoint
{

typedef UpdateTrajectoryNextPointRequest Request;
typedef UpdateTrajectoryNextPointResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct UpdateTrajectoryNextPoint
} // namespace panda_mpc


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::panda_mpc::UpdateTrajectoryNextPoint > {
  static const char* value()
  {
    return "2134cf5914ee56390a99d4f1731a484b";
  }

  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPoint&) { return value(); }
};

template<>
struct DataType< ::panda_mpc::UpdateTrajectoryNextPoint > {
  static const char* value()
  {
    return "panda_mpc/UpdateTrajectoryNextPoint";
  }

  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPoint&) { return value(); }
};


// service_traits::MD5Sum< ::panda_mpc::UpdateTrajectoryNextPointRequest> should match
// service_traits::MD5Sum< ::panda_mpc::UpdateTrajectoryNextPoint >
template<>
struct MD5Sum< ::panda_mpc::UpdateTrajectoryNextPointRequest>
{
  static const char* value()
  {
    return MD5Sum< ::panda_mpc::UpdateTrajectoryNextPoint >::value();
  }
  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPointRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::panda_mpc::UpdateTrajectoryNextPointRequest> should match
// service_traits::DataType< ::panda_mpc::UpdateTrajectoryNextPoint >
template<>
struct DataType< ::panda_mpc::UpdateTrajectoryNextPointRequest>
{
  static const char* value()
  {
    return DataType< ::panda_mpc::UpdateTrajectoryNextPoint >::value();
  }
  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPointRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::panda_mpc::UpdateTrajectoryNextPointResponse> should match
// service_traits::MD5Sum< ::panda_mpc::UpdateTrajectoryNextPoint >
template<>
struct MD5Sum< ::panda_mpc::UpdateTrajectoryNextPointResponse>
{
  static const char* value()
  {
    return MD5Sum< ::panda_mpc::UpdateTrajectoryNextPoint >::value();
  }
  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPointResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::panda_mpc::UpdateTrajectoryNextPointResponse> should match
// service_traits::DataType< ::panda_mpc::UpdateTrajectoryNextPoint >
template<>
struct DataType< ::panda_mpc::UpdateTrajectoryNextPointResponse>
{
  static const char* value()
  {
    return DataType< ::panda_mpc::UpdateTrajectoryNextPoint >::value();
  }
  static const char* value(const ::panda_mpc::UpdateTrajectoryNextPointResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PANDA_MPC_MESSAGE_UPDATETRAJECTORYNEXTPOINT_H