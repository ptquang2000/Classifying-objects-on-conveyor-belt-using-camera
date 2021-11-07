# Object classification projects using cameras

This project simulates a world which includes:

1. A conveyor belt
2. A camera sensor
3. Five models (Bear, Car, Giraffe, Helicopter, Train)

## Requirements

- Installing [rosbridge_server](https://wiki.ros.org/rosbridge_suite) package for visualization on browser with this [file](https://github.com/ptquang2000/Classifying-objects-on-conveyor-belt-using-camera/blob/main/html/index.html).
- Installing the [PyCoral API](https://coral.ai/docs/edgetpu/tflite-python/#inferencing-example) for running [Teachable machine](https://teachablemachine.withgoogle.com/) model.
- Installing [TensorFlow Lite for Python](https://www.tensorflow.org/lite/guide/python) for running tensor flow model.

## To start simulation world

```bash
catkin_make
source devel/setup.bash
roslaunch commander toy_detection.launch
```

## Demo

#### Video: https://www.youtube.com/watch?v=02AcFlf-MA0

![world](https://raw.githubusercontent.com/ptquang2000/Classifying-objects-on-conveyor-belt-using-camera/main/demo/world.png)

![result](https://raw.githubusercontent.com/ptquang2000/Classifying-objects-on-conveyor-belt-using-camera/main/demo/demo.png)
