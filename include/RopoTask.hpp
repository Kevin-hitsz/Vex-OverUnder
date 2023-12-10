#ifndef ROPO_TASK_HPP
#define ROPO_TASK_HPP


#include <iostream>
#include <map>      // pair
#include <string>       // string
#include "pros/rtos.hpp"
using namespace std;

namespace RopoTask
{
	extern map<string,pros::Task*>Task_Map;
    void CreateTask(string TaskName,void (*f)){//对某函数进行命名，并将其指针放入Task_Map中
        pros::Task* task_0 = new pros::Task(f);
        RopoTask::Task_Map.insert({TaskName,task_0});
    }
    void CreateTask(string TaskName,pros::Task* task_0){//对某任务指针进行命名，并将其指针放入Task_Map中
        RopoTask::Task_Map.insert({TaskName,task_0});
    }
    void SuspendTask(string TaskName){//通过名字，对任务进行挂起操作
        pros::Task* task_0;
        task_0 = RopoTask::Task_Map.find(TaskName)->second;
        (*task_0).suspend();
    }
    void ResumeTask(string TaskName){//通过名字，对任务进行恢复执行操作
        pros::Task* task_0;
        task_0 = RopoTask::Task_Map.find(TaskName)->second;
        (*task_0).resume();
    }
    void KillTask(string TaskName){//通过名字，对任务进行关闭操作，并将其指针从Task_Map去除
        pros::Task* task_0;
        task_0 = RopoTask::Task_Map.find(TaskName)->second;
        (*task_0).suspend();
        delete task_0;
        task_0 = nullptr;
        Task_Map.erase(TaskName);
    }

}

#endif //ROPO_TASK_HPP