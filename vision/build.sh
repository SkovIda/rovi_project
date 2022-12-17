#!/bin/bash
c++ -fPIC -pthread -std=c++17 \
	rws_sensor_plugin_example.cpp \
	-DQT_NO_VERSION_TAGGING \
	-L/usr/lib/x86_64-linux-gnu/ \
	-L/usr/lib/x86_64-linux-gnu/RobWork/rwplugins/ \
	-L/usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/ \
	-lsdurws_robworkstudioapp \
	-lsdurw_common \
	-lsdurw_core \
	-lsdurw_loaders \
	-lsdurw_models \
	-lsdurw_kinematics \
	-I/usr/include/eigen3/ -I/usr/include/robwork-2.5 \
	-I/usr/include/robworkstudio-2.5 \
	-I/usr/include/x86_64-linux-gnu/qt5/QtCore \
	-I/usr/include/x86_64-linux-gnu/qt6 \
	-I/usr/include/x86_64-linux-gnu/qt5/QtWidgets \
	-o vis.out #> log 2>&1 

