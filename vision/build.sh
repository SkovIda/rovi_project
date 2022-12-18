#!/bin/bash
c++ -fPIC -pthread -std=c++17 \
	rws_sensor_plugin_example.cpp \
	-DQT_NO_VERSION_TAGGING \
	-L/usr/lib/x86_64-linux-gnu/ \
	-L/usr/lib/x86_64-linux-gnu/RobWork/rwplugins/ \
	-L/usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/ \
	-L../SamplePlugin \
	-lsdurws_robworkstudioapp \
	-lsdurws \
	-lsdurw_common \
	-lsdurw_core \
	-lsdurw_loaders \
	-lsdurw_models \
	-lsdurw_kinematics \
	-lRoViPlugin \
	-I/usr/include/eigen3/ -I/usr/include/robwork-2.5 \
	-I/usr/include/robworkstudio-2.5 \
	-I/usr/include/x86_64-linux-gnu/qt5/QtCore \
	-I/usr/include/x86_64-linux-gnu/qt6 \
	-I/usr/include/x86_64-linux-gnu/qt5/QtWidgets \
	-I../SamplePlugin/src/ \
	-I../SamplePlugin/RoViPlugin_autogen/include/ \
	-I/usr/local/include/opencv4/ \
	-o vis.out #> log 2>&1 
