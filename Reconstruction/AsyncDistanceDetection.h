//
// Created by Kira on 24-4-19.
//

#ifndef ARDUCV_ASYNCDISTANCEDETECTION_H
#define ARDUCV_ASYNCDISTANCEDETECTION_H

#include "DistanceDetection.h"
#include <memory>

namespace ArduCV {

    class AsyncDistanceDetection {
    public:
        AsyncDistanceDetection(const CameraParamStereo &_param, const int &_cameraDeviceNo = 0) {
            detection = std::make_shared<DistanceDetection>(_param, _cameraDeviceNo);
        }

        bool startAsyncDetection(const std::function<void(DistanceDataFrame)> &subscriber, int nearThres) {
            if (taskRunning) {
                return false;
            } else {
                taskGroup.run([&]() {
                    detection->startContinuousDetection(subscriber, nearThres);
                });
                taskRunning = true;
                return true;
            }
        }

        void await(){
            taskGroup.wait();
        }

        void cancelTask() {
            if(taskRunning){
                detection->cancelTask();
                taskGroup.wait();
                taskRunning = false;
            }
        }

        bool isRunning(){return taskRunning;}

    private:
        bool taskRunning = false;
        std::shared_ptr<DistanceDetection> detection;
        tbb::task_group taskGroup;
    };

} // ArduCV

#endif //ARDUCV_ASYNCDISTANCEDETECTION_H
