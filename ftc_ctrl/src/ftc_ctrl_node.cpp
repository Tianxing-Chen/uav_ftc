// Copyright (C) 2021 Sihao Sun, RPG, University of Zurich, Switzerland
// Copyright (C) 2021 Davide Scaramuzza, RPG, University of Zurich, Switzerland
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <ros/ros.h>
#include "ftc_ctrl/ftc_ctrl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ftc_ctrl");
    ftc::NDICtrl NDI_ctrl;//默认构造函数委托有参构造实现，初始化了一个对象
    ros::spin();//更新迭代
    
    return 0;
}
