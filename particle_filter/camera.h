#ifndef CAMERA_H
#define CAMERA_H

void init_video_capture();

void destroy_video_capture();


void cameraCallback(ConstImageStampedPtr &msg);

#endif // CAMERA_H
