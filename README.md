# Motion-Aware Optical Camera Communication with Event Cameras

<p align="center">
    <a href="https://suhang99.github.io/">Hang Su</a> &emsp;&emsp; 
    <a href="https://www.linkedin.com/in/mgaoling/">Ling Gao</a>  &emsp;&emsp;
    <a href="https://www.linkedin.com/in/tao-liu-747327337/">Tao Liu</a> &emsp;&emsp;
    <a href="https://mpl.sist.shanghaitech.edu.cn/Director.html">Laurent Kneip</a> &emsp;&emsp;
</p>

<p align="center">
    <sup></sup>Mobile Perception Lab,
    <sup></sup>ShanghaiTech University
</p>

<p align="center">
    <a href="https://github.com/suhang99/EventOCC"><strong>Code</strong></a> |
    <a href="https://arxiv.org/pdf/2412.00816"><strong>arXiv</strong></a> |
    <a href="https://drive.google.com/file/d/1BW0kNI5JXVr535Ei5K5R-rVW_kXT2v6q/view?usp=sharing"><strong>Video</strong></a> | 
    <a href="https://drive.google.com/file/d/1AN-MyGDBzKnodQiOJwfY-3UyuNw-aiyB/view?usp=sharing"><strong>Test Data</strong></a>
</p>

The open-source implementation of "Motion-Aware Optical Camera Communication with Event Cameras".

<p align="center">
    <img src="assets/overview.png">
</p>

## Marker Generation
We have provided Python scripts to generate our dynamic markers.
### Dependencies
- OpenCV
- Numpy
- click
- alive_progress
- termcolor

### Example usage
```bash
python marker_generation/marker_generator.py [data.txt] --output_path [marker.mp4] --cell 16 --fps 60 --duration 30 
```
Please replace **[data.txt]** and **[marker.mp4]** with your input and output path.

Note that in our experiment for camera motion, we set the cell number to 16.

## Event-based OCC

The example data is available at [Google Drive](https://drive.google.com/file/d/1AN-MyGDBzKnodQiOJwfY-3UyuNw-aiyB/view?usp=sharing). The example data contain a rosbag, GT trajectory and the raw message to transmit.

The code implementation and usage are coming soon. 

## Acknowledgements

We would like to acknowledge the funding support provided by project 62250610225 by the Natural Science Foundation of China, as well as projects 22DZ1201900, 22ZR1441300, and dfycbj-1 by the Natural Science Foundation of Shanghai.