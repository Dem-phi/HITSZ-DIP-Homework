// Generated by gencpp from file dashgo_driver/SrvInt32Request.msg
// DO NOT EDIT!


#ifndef DASHGO_DRIVER_MESSAGE_SRVINT32REQUEST_H
#define DASHGO_DRIVER_MESSAGE_SRVINT32REQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dashgo_driver
{
template <class ContainerAllocator>
struct SrvInt32Request_
{
  typedef SrvInt32Request_<ContainerAllocator> Type;

  SrvInt32Request_()
    : arg1(0)  {
    }
  SrvInt32Request_(const ContainerAllocator& _alloc)
    : arg1(0)  {
  (void)_alloc;
    }



   typedef int32_t _arg1_type;
  _arg1_type arg1;





  typedef boost::shared_ptr< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> const> ConstPtr;

}; // struct SrvInt32Request_

typedef ::dashgo_driver::SrvInt32Request_<std::allocator<void> > SrvInt32Request;

typedef boost::shared_ptr< ::dashgo_driver::SrvInt32Request > SrvInt32RequestPtr;
typedef boost::shared_ptr< ::dashgo_driver::SrvInt32Request const> SrvInt32RequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dashgo_driver::SrvInt32Request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dashgo_driver::SrvInt32Request_<ContainerAllocator1> & lhs, const ::dashgo_driver::SrvInt32Request_<ContainerAllocator2> & rhs)
{
  return lhs.arg1 == rhs.arg1;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dashgo_driver::SrvInt32Request_<ContainerAllocator1> & lhs, const ::dashgo_driver::SrvInt32Request_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dashgo_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "064bdc12865ab344c8a048bd743c66ff";
  }

  static const char* value(const ::dashgo_driver::SrvInt32Request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x064bdc12865ab344ULL;
  static const uint64_t static_value2 = 0xc8a048bd743c66ffULL;
};

template<class ContainerAllocator>
struct DataType< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dashgo_driver/SrvInt32Request";
  }

  static const char* value(const ::dashgo_driver::SrvInt32Request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 arg1\n"
;
  }

  static const char* value(const ::dashgo_driver::SrvInt32Request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.arg1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SrvInt32Request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dashgo_driver::SrvInt32Request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dashgo_driver::SrvInt32Request_<ContainerAllocator>& v)
  {
    s << indent << "arg1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.arg1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DASHGO_DRIVER_MESSAGE_SRVINT32REQUEST_H
