// Generated by gencpp from file dashgo_tools/check_msgResult.msg
// DO NOT EDIT!


#ifndef DASHGO_TOOLS_MESSAGE_CHECK_MSGRESULT_H
#define DASHGO_TOOLS_MESSAGE_CHECK_MSGRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dashgo_tools
{
template <class ContainerAllocator>
struct check_msgResult_
{
  typedef check_msgResult_<ContainerAllocator> Type;

  check_msgResult_()
    : issuccess(false)  {
    }
  check_msgResult_(const ContainerAllocator& _alloc)
    : issuccess(false)  {
  (void)_alloc;
    }



   typedef uint8_t _issuccess_type;
  _issuccess_type issuccess;





  typedef boost::shared_ptr< ::dashgo_tools::check_msgResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dashgo_tools::check_msgResult_<ContainerAllocator> const> ConstPtr;

}; // struct check_msgResult_

typedef ::dashgo_tools::check_msgResult_<std::allocator<void> > check_msgResult;

typedef boost::shared_ptr< ::dashgo_tools::check_msgResult > check_msgResultPtr;
typedef boost::shared_ptr< ::dashgo_tools::check_msgResult const> check_msgResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dashgo_tools::check_msgResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dashgo_tools::check_msgResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dashgo_tools::check_msgResult_<ContainerAllocator1> & lhs, const ::dashgo_tools::check_msgResult_<ContainerAllocator2> & rhs)
{
  return lhs.issuccess == rhs.issuccess;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dashgo_tools::check_msgResult_<ContainerAllocator1> & lhs, const ::dashgo_tools::check_msgResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dashgo_tools

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dashgo_tools::check_msgResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dashgo_tools::check_msgResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dashgo_tools::check_msgResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e40869114e279d3e4fa137fdad5acd7d";
  }

  static const char* value(const ::dashgo_tools::check_msgResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe40869114e279d3eULL;
  static const uint64_t static_value2 = 0x4fa137fdad5acd7dULL;
};

template<class ContainerAllocator>
struct DataType< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dashgo_tools/check_msgResult";
  }

  static const char* value(const ::dashgo_tools::check_msgResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#result definition\n"
"bool issuccess\n"
;
  }

  static const char* value(const ::dashgo_tools::check_msgResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.issuccess);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct check_msgResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dashgo_tools::check_msgResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dashgo_tools::check_msgResult_<ContainerAllocator>& v)
  {
    s << indent << "issuccess: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.issuccess);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DASHGO_TOOLS_MESSAGE_CHECK_MSGRESULT_H
