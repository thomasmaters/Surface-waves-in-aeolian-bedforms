# Surface-waves-in-aeolian-bedforms
Implements the method described in the paper 'Surface waves in aeolian bedforms' by Qian-Hua Zhang.
This paper can be found https://www.sciencedirect.com/science/article/pii/S0375960108002211 .

The first implementation was made in Python, but the performance was not that good (+- 3s for a 50x50 grid).
The C++ implementation is a lot better ( +- 30ms for a 200x200 grid).

# Build-instructions (tested)
- With gcc 7.x or later
- With minimal -std=c++11
- With Opencv 2.0 or later

# Link instructions against:
- libopencv_core
- libopencv_highgui
- libopencv_imgproc
- libopencv_imgcodecs
