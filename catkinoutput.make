 kayo@kayo-mint  ~/Repos/ROSsell   doublegak  catkin_make
Base path: /home/kayo/Repos/ROSsell
Source space: /home/kayo/Repos/ROSsell/src
Build space: /home/kayo/Repos/ROSsell/build
Devel space: /home/kayo/Repos/ROSsell/devel
Install space: /home/kayo/Repos/ROSsell/install
####
#### Running command: "make cmake_check_build_system" in "/home/kayo/Repos/ROSsell/build"
####
####
#### Running command: "make -j8 -l8" in "/home/kayo/Repos/ROSsell/build"
####
Scanning dependencies of target exploreNode
Scanning dependencies of target obRec
[ 16%] Building CXX object finalproj/CMakeFiles/exploreNode.dir/src/exploreNode.cpp.o
[ 50%] Built target testNode
[ 66%] Building CXX object finalproj/CMakeFiles/obRec.dir/src/obRec.cpp.o
In file included from /opt/ros/kinetic/include/ros/serialization.h:37:0,
                 from /opt/ros/kinetic/include/ros/publisher.h:34,
                 from /opt/ros/kinetic/include/ros/node_handle.h:32,
                 from /opt/ros/kinetic/include/ros/ros.h:45,
                 from /home/kayo/Repos/ROSsell/src/finalproj/src/exploreNode.cpp:1:
/opt/ros/kinetic/include/ros/message_traits.h: In instantiation of ‘static const char* ros::message_traits::MD5Sum<M>::value(const M&) [with M = int]’:
/opt/ros/kinetic/include/ros/message_traits.h:255:102:   required from ‘const char* ros::message_traits::md5sum(const M&) [with M = int]’
/opt/ros/kinetic/include/ros/publisher.h:112:7:   required from ‘void ros::Publisher::publish(const M&) const [with M = int]’
/home/kayo/Repos/ROSsell/src/finalproj/src/exploreNode.cpp:58:43:   required from here
/opt/ros/kinetic/include/ros/message_traits.h:126:34: error: request for member ‘__getMD5Sum’ in ‘m’, which is of non-class type ‘const int’
     return m.__getMD5Sum().c_str();
                                  ^
/opt/ros/kinetic/include/ros/message_traits.h: In instantiation of ‘static const char* ros::message_traits::DataType<M>::value(const M&) [with M = int]’:
/opt/ros/kinetic/include/ros/message_traits.h:264:104:   required from ‘const char* ros::message_traits::datatype(const M&) [with M = int]’
/opt/ros/kinetic/include/ros/publisher.h:112:7:   required from ‘void ros::Publisher::publish(const M&) const [with M = int]’
/home/kayo/Repos/ROSsell/src/finalproj/src/exploreNode.cpp:58:43:   required from here
/opt/ros/kinetic/include/ros/message_traits.h:143:36: error: request for member ‘__getDataType’ in ‘m’, which is of non-class type ‘const int’
     return m.__getDataType().c_str();
                                    ^
finalproj/CMakeFiles/exploreNode.dir/build.make:62: recipe for target 'finalproj/CMakeFiles/exploreNode.dir/src/exploreNode.cpp.o' failed
make[2]: *** [finalproj/CMakeFiles/exploreNode.dir/src/exploreNode.cpp.o] Error 1
CMakeFiles/Makefile2:2525: recipe for target 'finalproj/CMakeFiles/exploreNode.dir/all' failed
make[1]: *** [finalproj/CMakeFiles/exploreNode.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
[ 83%] Linking CXX executable /home/kayo/Repos/ROSsell/devel/lib/finalproj/obRec
[ 83%] Built target obRec
Makefile:138: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j8 -l8" failed
