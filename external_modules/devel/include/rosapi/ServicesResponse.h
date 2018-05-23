// Generated by gencpp from file rosapi/ServicesResponse.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_SERVICESRESPONSE_H
#define ROSAPI_MESSAGE_SERVICESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosapi
{
template <class ContainerAllocator>
struct ServicesResponse_
{
  typedef ServicesResponse_<ContainerAllocator> Type;

  ServicesResponse_()
    : services()  {
    }
  ServicesResponse_(const ContainerAllocator& _alloc)
    : services(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _services_type;
  _services_type services;





  typedef boost::shared_ptr< ::rosapi::ServicesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosapi::ServicesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ServicesResponse_

typedef ::rosapi::ServicesResponse_<std::allocator<void> > ServicesResponse;

typedef boost::shared_ptr< ::rosapi::ServicesResponse > ServicesResponsePtr;
typedef boost::shared_ptr< ::rosapi::ServicesResponse const> ServicesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosapi::ServicesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosapi::ServicesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rosapi

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'HasHeader': False, 'IsFixedSize': False, 'IsMessage': True}
// {'rosapi': ['/home/ashis/botx_ws/PeanutHacks/external_modules/src/rosbridge_suite/rosapi/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct HasHeader< ::rosapi::ServicesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::ServicesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::ServicesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::ServicesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::ServicesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::ServicesResponse_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosapi::ServicesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e44a7e7bcb900acadbcc28b132378f0c";
  }

  static const char* value(const ::rosapi::ServicesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe44a7e7bcb900acaULL;
  static const uint64_t static_value2 = 0xdbcc28b132378f0cULL;
};

template<class ContainerAllocator>
struct DataType< ::rosapi::ServicesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosapi/ServicesResponse";
  }

  static const char* value(const ::rosapi::ServicesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosapi::ServicesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] services\n\
";
  }

  static const char* value(const ::rosapi::ServicesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosapi::ServicesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.services);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ServicesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosapi::ServicesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosapi::ServicesResponse_<ContainerAllocator>& v)
  {
    s << indent << "services[]" << std::endl;
    for (size_t i = 0; i < v.services.size(); ++i)
    {
      s << indent << "  services[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.services[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSAPI_MESSAGE_SERVICESRESPONSE_H
